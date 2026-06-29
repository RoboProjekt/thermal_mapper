"""Dynamischer TF-Broadcaster fuer SIYI Gimbal (Yaw/Pitch/Roll via UDP)."""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from thermal_mapper.gimbal_tf_utils import send_gimbal_transforms
from thermal_mapper.siyi_driver import SiyiGimbalDriver


class GimbalTfBroadcaster(Node):
    def __init__(self):
        super().__init__('gimbal_tf_broadcaster')

        self.declare_parameter('gimbal_ip', '192.168.133.25')
        self.declare_parameter('local_ip', '192.168.133.20')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('lens_mount_pitch_deg', 180.0)

        gimbal_ip = self.get_parameter('gimbal_ip').value
        local_ip = self.get_parameter('local_ip').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.lens_mount_pitch_deg = float(
            self.get_parameter('lens_mount_pitch_deg').value
        )

        self.broadcaster = TransformBroadcaster(self)

        try:
            self.driver = SiyiGimbalDriver(gimbal_ip=gimbal_ip, local_ip=local_ip)
            self.driver.start_telemetry()
            self.get_logger().info(
                f'SIYI Gimbal verbunden ({gimbal_ip}, lokal {local_ip})'
            )
        except OSError as exc:
            self.driver = None
            self.get_logger().error(f'Gimbal UDP-Binding fehlgeschlagen: {exc}')

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_transforms)

    def publish_transforms(self):
        if self.driver is None:
            yaw, pitch, roll = 0.0, 0.0, 0.0
        else:
            att = self.driver.current_attitude
            yaw = att['yaw']
            pitch = att['pitch']
            roll = att['roll']

        send_gimbal_transforms(
            self.broadcaster,
            self.get_clock().now().to_msg(),
            yaw,
            pitch,
            roll,
            self.lens_mount_pitch_deg,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.driver is not None:
            node.driver.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
