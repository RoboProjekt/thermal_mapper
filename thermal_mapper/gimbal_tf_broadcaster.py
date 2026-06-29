"""Dynamischer TF-Broadcaster fuer SIYI Gimbal (Yaw/Pitch/Roll via UDP)."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from thermal_mapper.siyi_driver import SiyiGimbalDriver


def euler_axis_quaternion(axis, angle_deg):
    """Quaternion fuer Rotation um eine Achse (x/y/z) in Grad."""
    angle_rad = math.radians(angle_deg)
    half = angle_rad * 0.5
    s = math.sin(half)
    c = math.cos(half)
    if axis == 'x':
        return (s, 0.0, 0.0, c)
    if axis == 'y':
        return (0.0, s, 0.0, c)
    return (0.0, 0.0, s, c)


def quat_multiply(q1, q2):
    """Hamilton-Produkt q1 * q2 (je (x, y, z, w))."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class GimbalTfBroadcaster(Node):
    def __init__(self):
        super().__init__('gimbal_tf_broadcaster')

        self.declare_parameter('gimbal_ip', '192.168.133.25')
        self.declare_parameter('local_ip', '192.168.133.20')
        self.declare_parameter('publish_rate_hz', 20.0)
        # SIYI-Montage: Linse zeigt entgegen TF-+X -> 180 deg um Y korrigiert Blickrichtung
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

        now = self.get_clock().now().to_msg()

        transforms = [
            (
                'gimbal_joint3_yaw', 'gimbal_joint2_roll',
                (-0.040, 0.0, 0.055), 'z', yaw
            ),
            (
                'gimbal_joint2_roll', 'gimbal_joint1_pitch',
                (0.045, 0.0, 0.0), 'x', roll
            ),
            (
                'gimbal_joint1_pitch', 'siyi_lens',
                (0.010, 0.0, 0.010), 'y', pitch
            ),
        ]

        for parent, child, translation, axis, angle_deg in transforms:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            qx, qy, qz, qw = euler_axis_quaternion(axis, angle_deg)
            if child == 'siyi_lens' and self.lens_mount_pitch_deg != 0.0:
                q_mount = euler_axis_quaternion(
                    'y', self.lens_mount_pitch_deg
                )
                qx, qy, qz, qw = quat_multiply(
                    (qx, qy, qz, qw), q_mount
                )
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.broadcaster.sendTransform(t)


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
