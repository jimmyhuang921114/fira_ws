import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class ColorObjectTracker(Node):
    def __init__(self):
        super().__init__('color_object_tracker')
        self.bridge = CvBridge()
        
        self.color_sub = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
        self.color_pub = self.create_publisher(String, '/color_sort', 10)

        self.color_range = {
            "Blue": {'min': [78, 89, 24], 'max': [115, 255, 255]},
            "Yellow": {'min': [13, 104, 55], 'max': [46, 221, 255]},
            "Green": {'min': [42, 52, 30], 'max': [77, 255, 255]}
        }

        self.bgr_color = {
            "Blue": (255, 0, 0),
            "Yellow": (0, 255, 255),
            "Green": (0, 255, 0)
        }

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detected_objects = []

        for color_name, hsv_range in self.color_range.items():
            lower = np.array(hsv_range['min'])
            upper = np.array(hsv_range['max'])

            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx = x + w // 2

                    detected_objects.append((cx, color_name))

                    cv2.rectangle(frame, (x, y), (x + w, y + h), self.bgr_color[color_name], 2)
                    cv2.putText(frame, color_name, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.bgr_color[color_name], 2)

        # Sort objects by x (left to right)
        detected_objects.sort(key=lambda obj: obj[0])
        sorted_colors = [obj[1] for obj in detected_objects]

        if sorted_colors:
            # Publish joined string array (like: "Green,Blue,Yellow")
            jsonfile = open("realsense/realsense/color_detect.py")
            data={sorted_colors}
            msg = String()
            json.dump(data,jsonfile)
            self.color_pub.publish(msg)
            self.get_logger().info(f'Published colors: {msg.data}')
            
        cv2.imshow("Color Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
