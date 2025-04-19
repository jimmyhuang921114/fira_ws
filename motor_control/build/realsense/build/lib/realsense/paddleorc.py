import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import paddle
from paddleocr import PaddleOCR
import logging


#
logging.getLogger('ppocr').setLevel(logging.ERROR)
# Force use of GPU
paddle.set_device("gpu")
paddle.set_flags({"FLAGS_cudnn_deterministic": True})

# GPU warm-up to avoid cuBLAS error
_ = paddle.matmul(paddle.to_tensor([[1.0, 2.0], [3.0, 4.0]]), paddle.to_tensor([[1.0], [1.0]]))

# Allowed characters to detect
TARGET_LETTERS = {"F", "I", "R", "A"}

def letterbox_resize(image, target_width, target_height, color=(0, 0, 0)):
    h, w = image.shape[:2]
    scale = min(target_width / w, target_height / h)
    new_w = int(w * scale)
    new_h = int(h * scale)

    resized = cv2.resize(image, (new_w, new_h))
    pad_w = target_width - new_w
    pad_h = target_height - new_h

    top = pad_h // 2
    bottom = pad_h - top
    left = pad_w // 2
    right = pad_w - left

    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return padded

class OCRNode(Node):
    def __init__(self):
        super().__init__('ocr_node')
        self.bridge = CvBridge()

        # Subscribe to color image topic
        self.subscription = self.create_subscription(
            Image, '/camera/color', self.image_callback, 10)

        # Publisher for recognized OCR results
        self.publisher = self.create_publisher(String, '/ocr_results', 10)

        self.get_logger().info('OCR Node Initialized')
        self.ocr = None  # Delay OCR initialization

    def image_callback(self, msg):
        try:
            # Lazy-load OCR model after CUDA initialized
            if self.ocr is None:
                self.ocr = PaddleOCR(
                    use_angle_cls=True,
                    lang="en",
                    det_db_box_thresh=0.3,
                    rec_algorithm='CRNN'
                )

            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Run OCR
            results = self.ocr.ocr(rgb_image, cls=True)
            detected_objects = []

            if results and isinstance(results, list) and results[0] is not None:
                for result in results:
                    for line in result:
                        # try:
                        points, (text, confidence) = line[0], line[1]
                        text = text.strip().upper()
                        if text in TARGET_LETTERS and confidence > 0.5:
                            x_center = int(sum([p[0] for p in points]) / 4)
                            y_center = int(sum([p[1] for p in points]) / 4)
                            detected_objects.append(f"{text},{x_center},{y_center}")

                            # Draw bounding box and label on image
                            points_np = np.array(points, np.int32).reshape((-1, 1, 2))
                            cv2.polylines(cv_image, [points_np], isClosed=True, color=(0, 255, 0), thickness=2)
                            cv2.putText(
                                cv_image, f"{text} ({confidence:.2f})", (x_center, y_center),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA
                            )
                        # except Exception as e:
                        #     self.get_logger().error(f"Failed to parse OCR result: {e}")

            # Publish result string
            if detected_objects:
                output_msg = String()
                output_msg.data = ";".join(detected_objects)
                self.publisher.publish(output_msg)
                # self.get_logger().info(f"OCR result: {output_msg.data}")

            # Resize to 1280x720 using padding (no distortion)
            cv_image_resized = letterbox_resize(cv_image, 1280, 720)
            cv2.namedWindow("OCR Detection", cv2.WINDOW_NORMAL)
            cv2.imshow("OCR Detection", cv_image_resized)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OCRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
