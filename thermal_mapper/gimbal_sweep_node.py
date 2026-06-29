"""Automatischer horizontaler Gimbal-Sweep mit Closed-Loop (Geschwindigkeit 0x07)."""

import csv
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from thermal_mapper.siyi_driver import SiyiGimbalDriver


def parse_yaw_targets(text):
    """Komma-getrennte Yaw-Ziele parsen, z.B. '180,-180,0'."""
    parts = [p.strip() for p in str(text).split(',') if p.strip()]
    if not parts:
        raise ValueError('yaw_targets_deg darf nicht leer sein')
    return [float(p) for p in parts]


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('true', '1', 'yes')


def angle_diff(target_deg, current_deg):
    """Kuerzester Winkel target - current in Grad."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


class SweepState(Enum):
    WAIT_TELEMETRY = 'WAIT_TELEMETRY'
    CALIBRATE_DIR = 'CALIBRATE_DIR'
    GO_TARGET = 'GO_TARGET'
    SETTLING = 'SETTLING'
    DONE = 'DONE'
    FAILED = 'FAILED'


class GimbalSweepNode(Node):
    def __init__(self):
        super().__init__('gimbal_sweep_node')

        self.declare_parameter('gimbal_ip', '192.168.133.25')
        self.declare_parameter('local_ip', '192.168.133.20')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('yaw_targets_deg', '180,-180,0')
        self.declare_parameter('use_relative_yaw', True)
        self.declare_parameter('yaw_speed', 20)
        self.declare_parameter('tolerance_deg', 2.0)
        self.declare_parameter('step_timeout_s', 30.0)
        self.declare_parameter('settle_time_s', 1.0)
        self.declare_parameter('startup_delay_s', 3.0)
        self.declare_parameter('calibration_duration_s', 0.6)
        self.declare_parameter('invert_yaw', 'auto')
        self.declare_parameter('csv_log_path', '/tmp/gimbal_sweep_log.csv')
        self.declare_parameter('auto_start', True)

        self.yaw_speed_cmd = int(self.get_parameter('yaw_speed').value)
        self.tolerance_deg = float(self.get_parameter('tolerance_deg').value)
        self.step_timeout_s = float(self.get_parameter('step_timeout_s').value)
        self.settle_time_s = float(self.get_parameter('settle_time_s').value)
        self.startup_delay_s = float(self.get_parameter('startup_delay_s').value)
        self.calibration_duration_s = float(
            self.get_parameter('calibration_duration_s').value
        )
        self.use_relative_yaw = parse_bool(
            self.get_parameter('use_relative_yaw').value
        )
        self.csv_log_path = self.get_parameter('csv_log_path').value
        self.auto_start = parse_bool(self.get_parameter('auto_start').value)

        invert_param = str(self.get_parameter('invert_yaw').value).lower()
        if invert_param in ('true', '1', 'yes'):
            self.invert_yaw = 1
            self.invert_yaw_auto = False
        elif invert_param in ('false', '0', 'no'):
            self.invert_yaw = -1
            self.invert_yaw_auto = False
        else:
            self.invert_yaw = 1
            self.invert_yaw_auto = True

        self.raw_targets = parse_yaw_targets(
            self.get_parameter('yaw_targets_deg').value
        )
        self.absolute_targets = []
        self.step_reports = []

        self.state = SweepState.WAIT_TELEMETRY
        self.state_enter_time = time.monotonic()
        self.step_index = 0
        self.current_target = None
        self.start_yaw = 0.0
        self.home_yaw = 0.0
        self.command_yaw_speed = 0
        self.calibration_yaw_start = 0.0
        self._last_telemetry_yaw = None
        self._telemetry_alive = False
        self._csv_file = None
        self._csv_writer = None

        self.status_pub = self.create_publisher(String, '/gimbal_sweep/status', 10)
        self.yaw_current_pub = self.create_publisher(
            Float64, '/gimbal_sweep/yaw_current', 10
        )
        self.yaw_target_pub = self.create_publisher(
            Float64, '/gimbal_sweep/yaw_target', 10
        )

        gimbal_ip = self.get_parameter('gimbal_ip').value
        local_ip = self.get_parameter('local_ip').value

        try:
            self.driver = SiyiGimbalDriver(
                gimbal_ip=gimbal_ip,
                local_ip=local_ip,
            )
            self.driver.start_telemetry()
            self.get_logger().info(
                f'Gimbal-Sweep verbunden ({gimbal_ip}, lokal {local_ip})'
            )
        except OSError as exc:
            self.driver = None
            self.state = SweepState.FAILED
            self.get_logger().error(f'Gimbal UDP-Binding fehlgeschlagen: {exc}')

        rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.control_tick)
        self._open_csv_log()

        if self.auto_start and self.driver is not None:
            self.get_logger().info(
                f'Sweep startet in {self.startup_delay_s:.1f} s '
                f'(Ziele: {self.raw_targets}, relativ={self.use_relative_yaw})'
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
            self._csv_file = None
            self._csv_writer = None

    def current_yaw(self):
        if self.driver is None:
            return 0.0
        return float(self.driver.current_attitude['yaw'])

    def _set_state(self, new_state):
        self.state = new_state
        self.state_enter_time = time.monotonic()
        self.get_logger().info(f'State -> {new_state.value}')

    def _build_absolute_targets(self):
        if self.use_relative_yaw:
            self.absolute_targets = [
                self.home_yaw + offset for offset in self.raw_targets
            ]
        else:
            self.absolute_targets = list(self.raw_targets)

    def _publish_status(self, extra=''):
        yaw = self.current_yaw()
        target = self.current_target
        error = angle_diff(target, yaw) if target is not None else 0.0
        msg = (
            f'state={self.state.value} '
            f'step={self.step_index + 1}/{len(self.absolute_targets)} '
            f'target={target if target is not None else "n/a"} '
            f'yaw={yaw:.1f} error={error:.1f} '
            f'cmd_speed={self.command_yaw_speed}'
        )
        if extra:
            msg += f' {extra}'
        self.status_pub.publish(String(data=msg))

        self.yaw_current_pub.publish(Float64(data=yaw))
        if target is not None:
            self.yaw_target_pub.publish(Float64(data=target))

        if self._csv_writer is not None:
            self._csv_writer.writerow([
                f'{time.monotonic():.3f}',
                self.state.value,
                self.step_index + 1,
                f'{target:.2f}' if target is not None else '',
                f'{yaw:.2f}',
                self.command_yaw_speed,
                f'{error:.2f}',
            ])
            self._csv_file.flush()

    def _stop_gimbal(self):
        self.command_yaw_speed = 0
        if self.driver is not None:
            self.driver.set_gimbal_speed(0, 0)

    def _start_next_target(self):
        if self.step_index >= len(self.absolute_targets):
            self._finish_sweep()
            return
        self.current_target = self.absolute_targets[self.step_index]
        self._set_state(SweepState.GO_TARGET)
        self.get_logger().info(
            f'Schritt {self.step_index + 1}/{len(self.absolute_targets)}: '
            f'Ziel-Yaw {self.current_target:.1f} deg'
        )

    def _finish_step(self, reached_yaw, timed_out=False):
        target = self.current_target
        error = angle_diff(target, reached_yaw)
        duration = time.monotonic() - self.state_enter_time
        report = {
            'step': self.step_index + 1,
            'target': target,
            'reached': reached_yaw,
            'error': error,
            'duration_s': duration,
            'timed_out': timed_out,
        }
        self.step_reports.append(report)

        status = 'TIMEOUT' if timed_out else 'OK'
        self.get_logger().info(
            f'Schritt {report["step"]} {status}: '
            f'Ziel={target:.1f} erreicht={reached_yaw:.1f} '
            f'Fehler={error:.1f} deg Dauer={duration:.1f}s'
        )

        self._stop_gimbal()
        self.step_index += 1
        self._set_state(SweepState.SETTLING)

    def _finish_sweep(self):
        self._stop_gimbal()
        self._set_state(SweepState.DONE)
        self.get_logger().info('=' * 55)
        self.get_logger().info('SWEEP ABGESCHLOSSEN – Zusammenfassung:')
        self.get_logger().info(
            f'  Start-Yaw: {self.start_yaw:.1f} deg  '
            f'End-Yaw: {self.current_yaw():.1f} deg'
        )
        for rep in self.step_reports:
            flag = 'TIMEOUT' if rep['timed_out'] else 'OK'
            self.get_logger().info(
                f'  Schritt {rep["step"]}: {flag}  '
                f'Ziel={rep["target"]:.1f}  Ist={rep["reached"]:.1f}  '
                f'Fehler={rep["error"]:.1f} deg  '
                f'{rep["duration_s"]:.1f}s'
            )
        self.get_logger().info(f'  CSV: {self.csv_log_path}')
        self.get_logger().info('=' * 55)

    def control_tick(self):
        if self.driver is None:
            self._publish_status('driver=missing')
            return

        yaw = self.current_yaw()
        if self._last_telemetry_yaw is None:
            self._last_telemetry_yaw = yaw
        elif abs(yaw - self._last_telemetry_yaw) > 0.01:
            self._telemetry_alive = True
        self._last_telemetry_yaw = yaw

        if self.state == SweepState.WAIT_TELEMETRY:
            elapsed = time.monotonic() - self.state_enter_time
            if elapsed < self.startup_delay_s:
                self._publish_status('waiting_startup')
                return
            if not self._telemetry_alive:
                self.get_logger().warn(
                    'Keine Telemetrie-Aenderung – starte trotzdem',
                    throttle_duration_sec=5.0,
                )
            self.home_yaw = yaw
            self.start_yaw = yaw
            self._build_absolute_targets()
            self.get_logger().info(
                f'Start-Yaw={self.start_yaw:.1f} deg, '
                f'Absolute Ziele={[round(t, 1) for t in self.absolute_targets]}'
            )
            self._set_state(SweepState.CALIBRATE_DIR)
            self.calibration_yaw_start = yaw
            self.command_yaw_speed = self.yaw_speed_cmd * self.invert_yaw
            self.driver.set_gimbal_speed(self.command_yaw_speed, 0)
            return

        if self.state == SweepState.CALIBRATE_DIR:
            elapsed = time.monotonic() - self.state_enter_time
            if elapsed < self.calibration_duration_s:
                self._publish_status('calibrating')
                return

            self._stop_gimbal()
            delta = angle_diff(self.current_yaw(), self.calibration_yaw_start)
            if self.invert_yaw_auto:
                if delta > 0.5:
                    self.invert_yaw = 1
                elif delta < -0.5:
                    self.invert_yaw = -1
                else:
                    self.invert_yaw = 1
                    self.get_logger().warn(
                        'Kalibrierung unklar (kaum Bewegung) – invert_yaw=1'
                    )
                self.get_logger().info(
                    f'Yaw-Richtung kalibriert: delta={delta:.1f} deg, '
                    f'invert_yaw={self.invert_yaw}'
                )
            self.step_index = 0
            self._start_next_target()
            return

        if self.state == SweepState.GO_TARGET:
            target = self.current_target
            error = angle_diff(target, yaw)
            elapsed = time.monotonic() - self.state_enter_time

            if abs(error) <= self.tolerance_deg:
                self._finish_step(yaw, timed_out=False)
                return

            if elapsed >= self.step_timeout_s:
                self.get_logger().warn(
                    f'Schritt-Timeout nach {elapsed:.1f}s, '
                    f'Ziel={target:.1f} Ist={yaw:.1f}'
                )
                self._finish_step(yaw, timed_out=True)
                return

            direction = 1 if error > 0 else -1
            self.command_yaw_speed = direction * self.yaw_speed_cmd * self.invert_yaw
            self.driver.set_gimbal_speed(self.command_yaw_speed, 0)
            self._publish_status()
            return

        if self.state == SweepState.SETTLING:
            elapsed = time.monotonic() - self.state_enter_time
            self._publish_status('settling')
            if elapsed >= self.settle_time_s:
                self._start_next_target()
            return

        if self.state in (SweepState.DONE, SweepState.FAILED):
            self._publish_status('finished')
            return

    def destroy_node(self):
        self._stop_gimbal()
        if self.driver is not None:
            self.driver.stop()
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
