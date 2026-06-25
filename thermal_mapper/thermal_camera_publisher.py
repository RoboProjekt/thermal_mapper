"""RTSP Thermal-Kamera Publisher (TCP) als sensor_msgs/Image."""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# FFmpeg TCP erzwingen (UDP fuehrt zu Frame-Fehlern)
os.environ.setdefault('OPENCV_FFMPEG_LOGLEVEL', 'quiet')
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')


class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__('thermal_camera_publisher')

        self.declare_parameter('rtsp_url', 'rtsp://192.168.133.25:8554/video1')
        self.declare_parameter('topic', '/siyi/thermal/image_raw')
        self.declare_parameter('frame_id', 'siyi_lens')
        self.declare_parameter('publish_rate_hz', 25.0)

        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self.publisher = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()
        self.cap = None

        self._connect_camera()
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_frame)

    def _connect_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if self.cap.isOpened():
            self.get_logger().info(f'RTSP verbunden: {self.rtsp_url}')
        else:
            self.get_logger().warn(f'RTSP nicht erreichbar: {self.rtsp_url}')

    def publish_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self._connect_camera()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame verloren, Reconnect...')
            self._connect_camera()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
