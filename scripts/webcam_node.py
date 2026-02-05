#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2

class CamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        # Use V4L2 backend explicitly for better compatibility
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        # Read a test frame
        ret, frame = self.cap.read()
        if ret:
            self.get_logger().info(f'Camera test OK: {frame.shape}, mean={frame.mean():.1f}')
        else:
            self.get_logger().error('Camera opened but cannot read frames!')

        self.timer = self.create_timer(1/30, self.timer_cb)
        self.get_logger().info('Webcam started (640x480 @ 30fps)')

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame', throttle_duration_sec=5.0)
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
