#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class LaneTrackerNode(Node):
    def __init__(self):
        super().__init__('lane_tracker_node')

        self.declare_parameter('kp', 0.005)
        self.declare_parameter('base_speed', 0.2)
        
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.base_speed = self.get_parameter('base_speed').get_parameter_value().double_value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/lane_tracker/debug_image', 10)

        self.get_logger().info('Lane Tracker Node sikeresen elindult.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Hiba: {e}')
            return

        height, width, _ = cv_image.shape

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 55, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = cv2.bitwise_or(mask_white, mask_yellow)

        roi_mask = np.zeros_like(mask)
        roi_vertices = np.array([
            [(0, height), (0, int(height * 0.6)), (width, int(height * 0.6)), (width, height)]
        ], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        masked_image = cv2.bitwise_and(mask, roi_mask)

        M = cv2.moments(masked_image)
        cmd = Twist()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            image_center_x = width // 2
            error = cx - image_center_x

            cmd.linear.x = self.base_speed
            cmd.angular.z = -float(error) * self.kp

            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(cv_image, (image_center_x, 0), (image_center_x, height), (255, 0, 0), 2)
            cv2.putText(cv_image, f"Hiba: {error} px", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.1
            cv2.putText(cv_image, "Sáv nem található!", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        self.cmd_pub.publish(cmd)

        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
