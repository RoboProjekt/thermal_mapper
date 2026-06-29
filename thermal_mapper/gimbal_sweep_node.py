"""Automatischer horizontaler Gimbal-Sweep mit Closed-Loop (Geschwindigkeit 0x07)."""

import csv
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from thermal_mapper.gimbal_sweep_controller import (
    GimbalSweepController,
    describe_sweep_plan,
    parse_bool,
    parse_yaw_list,
)
from thermal_mapper.siyi_driver import SiyiGimbalDriver


class GimbalSweepNode(Node):
    def __init__(self):
        super().__init__('gimbal_sweep_node')

        self.declare_parameter('gimbal_ip', '192.168.133.25')
        self.declare_parameter('local_ip', '192.168.133.20')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('yaw_targets_deg', '180,-180,0')
        self.declare_parameter('target_mode', 'delta')
        self.declare_parameter('yaw_speed', 20)
        self.declare_parameter('tolerance_deg', 2.0)
        self.declare_parameter('step_timeout_s', 30.0)
        self.declare_parameter('settle_time_s', 1.0)
        self.declare_parameter('startup_delay_s', 3.0)
        self.declare_parameter('calibration_duration_s', 0.6)
        self.declare_parameter('invert_yaw', 'auto')
        self.declare_parameter('csv_log_path', '/tmp/gimbal_sweep_log.csv')
        self.declare_parameter('auto_start', True)

        yaw_targets = parse_yaw_list(self.get_parameter('yaw_targets_deg').value)
        target_mode = str(self.get_parameter('target_mode').value).lower()
        self.csv_log_path = self.get_parameter('csv_log_path').value
        self.auto_start = parse_bool(self.get_parameter('auto_start').value)

        self.status_pub = self.create_publisher(String, '/gimbal_sweep/status', 10)
        self.yaw_current_pub = self.create_publisher(
            Float64, '/gimbal_sweep/yaw_current', 10
        )
        self.yaw_target_pub = self.create_publisher(
            Float64, '/gimbal_sweep/yaw_target', 10
        )

        gimbal_ip = self.get_parameter('gimbal_ip').value
        local_ip = self.get_parameter('local_ip').value

        self.controller = None
        self._csv_file = None
        self._csv_writer = None
        self._plan_logged = False
        self._finish_logged = False

        try:
            driver = SiyiGimbalDriver(
                gimbal_ip=gimbal_ip,
                local_ip=local_ip,
            )
            driver.start_telemetry()
            self.controller = GimbalSweepController(
                driver=driver,
                yaw_targets=yaw_targets,
                target_mode=target_mode,
                yaw_speed=int(self.get_parameter('yaw_speed').value),
                tolerance_deg=float(self.get_parameter('tolerance_deg').value),
                step_timeout_s=float(self.get_parameter('step_timeout_s').value),
                settle_time_s=float(self.get_parameter('settle_time_s').value),
                startup_delay_s=float(self.get_parameter('startup_delay_s').value),
                calibration_duration_s=float(
                    self.get_parameter('calibration_duration_s').value
                ),
                invert_yaw=str(self.get_parameter('invert_yaw').value),
            )
            self.get_logger().info(
                f'Gimbal-Sweep verbunden ({gimbal_ip}, lokal {local_ip})'
            )
        except OSError as exc:
            self.get_logger().error(f'Gimbal UDP-Binding fehlgeschlagen: {exc}')

        rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.control_tick)
        self._open_csv_log()

        if self.auto_start and self.controller is not None:
            self.get_logger().info(
                f'Sweep startet in {self.controller.startup_delay_s:.1f} s '
                f'(Modus={target_mode}, Ziele={yaw_targets})'
            )

    def _open_csv_log(self):
        try:
            self._csv_file = open(self.csv_log_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'time_s', 'state', 'step', 'target_yaw', 'telemetry_yaw',
                'commanded_yaw_speed', 'error_deg',
            ])
            self._csv_file.flush()
            self.get_logger().info(f'CSV-Log: {self.csv_log_path}')
        except OSError as exc:
            self.get_logger().warn(f'CSV-Log nicht schreibbar: {exc}')

    def _log_plan_once(self, home_yaw):
        if self._plan_logged:
            return
        self._plan_logged = True
        self.get_logger().info(f'Start-Yaw: {home_yaw:.1f} deg')
        self.get_logger().info('Sweep-Plan:')
        for line in describe_sweep_plan(
            self.controller.target_mode,
            self.controller.yaw_targets,
            home_yaw,
        ):
            self.get_logger().info(line)

    def _publish_status(self, status, extra=''):
        target = status['target']
        msg = (
            f"state={status['state']} "
            f"step={status['step']}/{status['total_steps']} "
            f"target={target if target is not None else 'n/a'} "
            f"yaw={status['yaw']:.1f} error={status['error']:.1f} "
            f"cmd_speed={status['cmd_speed']}"
        )
        if extra:
            msg += f' {extra}'
        self.status_pub.publish(String(data=msg))
        self.yaw_current_pub.publish(Float64(data=status['yaw']))
        if target is not None:
            self.yaw_target_pub.publish(Float64(data=target))

        if self._csv_writer is not None:
            self._csv_writer.writerow([
                f'{time.monotonic():.3f}',
                status['state'],
                status['step'],
                f'{target:.2f}' if target is not None else '',
                f'{status["yaw"]:.2f}',
                status['cmd_speed'],
                f'{status["error"]:.2f}',
            ])
            self._csv_file.flush()

    def control_tick(self):
        if self.controller is None:
            self.status_pub.publish(String(data='state=FAILED driver=missing'))
            return

        prev_step_count = len(self.controller.step_reports)
        status = self.controller.tick()

        if status['state'] == 'CALIBRATE_DIR' and not self._plan_logged:
            self._log_plan_once(self.controller.home_yaw)

        self._publish_status(status)

        if len(self.controller.step_reports) > prev_step_count:
            rep = self.controller.step_reports[-1]
            flag = 'TIMEOUT' if rep['timed_out'] else 'OK'
            self.get_logger().info(
                f'Schritt {rep["step"]} {flag}: '
                f'Ziel={rep["target"]:.1f} Ist={rep["reached"]:.1f} '
                f'Fehler={rep["error"]:.1f} deg'
            )

        if self.controller.is_finished() and not self._finish_logged:
            self._finish_logged = True
            self.get_logger().info('=' * 55)
            self.get_logger().info('SWEEP ABGESCHLOSSEN')
            self.get_logger().info(
                f'  Start-Yaw: {self.controller.start_yaw:.1f} deg  '
                f'End-Yaw: {self.controller.current_yaw():.1f} deg'
            )
            for rep in self.controller.step_reports:
                flag = 'TIMEOUT' if rep['timed_out'] else 'OK'
                self.get_logger().info(
                    f'  Schritt {rep["step"]}: {flag}  '
                    f'Ziel={rep["target"]:.1f}  Ist={rep["reached"]:.1f}  '
                    f'Fehler={rep["error"]:.1f} deg'
                )
            self.get_logger().info(f'  CSV: {self.csv_log_path}')
            self.get_logger().info('=' * 55)

    def destroy_node(self):
        if self.controller is not None:
            self.controller.stop()
            self.controller.driver.stop()
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
