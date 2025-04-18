import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import paddle
from paddleocr import PaddleOCR

# 强制使用 GPU
paddle.set_device("gpu")
paddle.set_flags({"FLAGS_cudnn_deterministic": True})

# GPU 预热（触发 CUDA 初始化，避免 cuBLAS 出错）
_ = paddle.matmul(paddle.to_tensor([[1.0, 2.0], [3.0, 4.0]]), paddle.to_tensor([[1.0], [1.0]]))

# 允许识别的字母
TARGET_LETTERS = {"F", "I", "R", "A"}

class OCRNode(Node):
    def __init__(self):
        super().__init__('ocr_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/color', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/ocr_results', 10)
        self.get_logger().info('OCR Node Initialized')
        self.ocr = None  # OCR 初始化延后
    
    def image_callback(self, msg):
        try:
            # 延迟 OCR 初始化，确保 CUDA 初始化完成
            if self.ocr is None:
                self.ocr = PaddleOCR(use_angle_cls=True, lang="en", det_db_box_thresh=0.3, rec_algorithm='CRNN')

            # 转换 ROS Image 到 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # OCR 识别
            results = self.ocr.ocr(rgb_image, cls=True)
            detected_objects = []

            if results and isinstance(results, list) and results[0] is not None:
                for result in results:
                    for line in result:
                        try:
                            points, (text, confidence) = line[0], line[1]
                            text = text.strip().upper()
                            if text in TARGET_LETTERS and confidence > 0.5:
                                x_center = int(sum([p[0] for p in points]) / 4)
                                y_center = int(sum([p[1] for p in points]) / 4)
                                detected_objects.append(f"{text},{x_center},{y_center}")
                                points_np = np.array(points, np.int32).reshape((-1, 1, 2))
                                cv2.polylines(cv_image, [points_np], isClosed=True, color=(0, 255, 0), thickness=2)
                                cv2.putText(cv_image, f"{text} ({confidence:.2f})", (x_center, y_center),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                        except Exception as e:
                            self.get_logger().error(f"解析 OCR 结果出错: {e}")

            if detected_objects:
                output_msg = String()
                output_msg.data = ";".join(detected_objects)
                self.publisher.publish(output_msg)
                self.get_logger().info(f"检测结果: {output_msg.data}")

            cv2.imshow("OCR Detection", cv_image)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"处理图像出错: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OCRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
