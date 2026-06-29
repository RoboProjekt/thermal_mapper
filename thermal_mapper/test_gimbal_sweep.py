#!/usr/bin/env python3
"""
Manueller Gimbal-Test mit absoluten Yaw-Winkeln und TF-Publikation.

Steuerung + TF laufen im selben Node (ein UDP-Treiber, TF bewegt sich in RViz).
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from thermal_mapper.gimbal_control import (
    calibrate_yaw_direction,
    move_to_absolute_yaw,
    normalize_angle,
)
from thermal_mapper.gimbal_tf_utils import send_gimbal_transforms
from thermal_mapper.siyi_driver import SiyiGimbalDriver


class GimbalManualTestNode(Node):
    def __init__(self):
        super().__init__('gimbal_manual_test')

        self.declare_parameter('gimbal_ip', '192.168.133.25')
        self.declare_parameter('local_ip', '192.168.133.20')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('lens_mount_pitch_deg', 180.0)
        self.declare_parameter('yaw_speed', 15)
        self.declare_parameter('tolerance_deg', 2.0)
        self.declare_parameter('step_timeout_s', 30.0)

        self.yaw_speed = int(self.get_parameter('yaw_speed').value)
        self.tolerance_deg = float(self.get_parameter('tolerance_deg').value)
        self.step_timeout_s = float(self.get_parameter('step_timeout_s').value)
        self.lens_mount_pitch_deg = float(
            self.get_parameter('lens_mount_pitch_deg').value
        )
        self.invert_yaw = 1
        self._running = True

        gimbal_ip = self.get_parameter('gimbal_ip').value
        local_ip = self.get_parameter('local_ip').value

        self.broadcaster = TransformBroadcaster(self)
        self.driver = SiyiGimbalDriver(gimbal_ip=gimbal_ip, local_ip=local_ip)
        self.driver.start_telemetry()

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate_hz, self.publish_tf)

        self.get_logger().info(
            f'Gimbal verbunden ({gimbal_ip}, lokal {local_ip}), TF aktiv'
        )

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

    def publish_tf(self):
        att = self.driver.current_attitude
        send_gimbal_transforms(
            self.broadcaster,
            self.get_clock().now().to_msg(),
            att['yaw'],
            att['pitch'],
            att['roll'],
            self.lens_mount_pitch_deg,
        )

    def wait_telemetry(self, seconds=1.0):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.spin_once()
            time.sleep(0.05)

    def current_yaw(self):
        return float(self.driver.current_attitude['yaw'])

    def calibrate(self):
        self.get_logger().info('Kalibriere Drehrichtung ...')
        self.invert_yaw = calibrate_yaw_direction(
            self.driver,
            self.yaw_speed,
            tick_callback=self.spin_once,
        )
        self.get_logger().info(f'Drehrichtung: invert_yaw={self.invert_yaw}')

    def move_to(self, abs_target):
        abs_target = normalize_angle(abs_target)
        self.get_logger().info(f'Fahre zu absolut {abs_target:+.1f} deg ...')

        def on_update(info):
            sys.stdout.write(
                f"\r  -> Ziel {info['target']:+6.1f} deg  "
                f"Ist {info['current']:+6.1f} deg  "
                f"Fehler {info['error']:+5.1f} deg  "
                f"{info['elapsed']:4.1f}s  "
            )
            sys.stdout.flush()

        result = move_to_absolute_yaw(
            self.driver,
            abs_target,
            yaw_speed=self.yaw_speed,
            tolerance_deg=self.tolerance_deg,
            step_timeout_s=self.step_timeout_s,
            invert_yaw=self.invert_yaw,
            on_update=on_update,
            tick_callback=self.spin_once,
        )
        print()
        flag = 'TIMEOUT' if result['timed_out'] else 'OK'
        self.get_logger().info(
            f'[{flag}] Ziel {result["target"]:+.1f} deg, '
            f'erreicht {result["reached"]:+.1f} deg, '
            f'Fehler {result["error"]:+.1f} deg'
        )

    def print_help(self):
        print()
        print('=' * 58)
        print('Manueller Gimbal-Test (absolute Yaw-Winkel + TF)')
        print('Eingabe = Zielwinkel in Grad, z.B. 0, 90, -180')
        print('=' * 58)
        print('  <zahl>   Absoluter Ziel-Yaw')
        print('  s        Status')
        print('  q        Beenden')
        print()

    def input_loop(self):
        self.print_help()
        while self._running and rclpy.ok():
            print(f'  Aktueller Yaw: {self.current_yaw():+7.1f} deg')
            try:
                raw = input('Ziel-Yaw absolut (Grad): ').strip()
            except EOFError:
                break

            cmd = raw.lower()
            if cmd in ('q', 'quit', 'exit'):
                self._running = False
                break
            if cmd in ('', 's', 'status'):
                continue

            try:
                self.move_to(float(raw))
            except ValueError:
                print('Ungueltige Eingabe. Zahl, s oder q.')

    def shutdown(self):
        self._running = False
        self.driver.set_gimbal_speed(0, 0)
        self.driver.stop()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalManualTestNode()

    input_thread = threading.Thread(target=node.input_loop, daemon=True)
    input_thread.start()

    try:
        node.wait_telemetry()
        node.calibrate()
        while node._running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
