"""Gemeinsame Sweep-Logik fuer gimbal_sweep_node und test_gimbal_sweep."""

import time
from enum import Enum


def normalize_angle(deg):
    """Winkel auf [-180, 180] Grad normalisieren."""
    return (deg + 180.0) % 360.0 - 180.0


def angle_diff(target_deg, current_deg):
    """Kuerzester Winkel target - current in Grad."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def relative_yaw(home_abs, current_abs):
    """Aktueller Yaw relativ zur Referenz 'vorn' (home = 0 deg)."""
    return angle_diff(current_abs, home_abs)


def absolute_from_relative(home_abs, rel_deg):
    """Relativen Zielwinkel in absoluten Gimbal-Yaw umrechnen."""
    return normalize_angle(home_abs + rel_deg)


def calibrate_yaw_direction(driver, yaw_speed, duration_s=0.5):
    """
    Kurzer Testimpuls: liefert +1 oder -1 als Korrekturfaktor fuer set_gimbal_speed.
    """
    start = float(driver.current_attitude['yaw'])
    driver.set_gimbal_speed(yaw_speed, 0)
    time.sleep(duration_s)
    driver.set_gimbal_speed(0, 0)
    time.sleep(0.2)
    delta = angle_diff(float(driver.current_attitude['yaw']), start)
    if delta > 0.5:
        return 1
    if delta < -0.5:
        return -1
    return 1


def move_to_absolute_yaw(
    driver,
    abs_target_deg,
    yaw_speed=15,
    tolerance_deg=2.0,
    step_timeout_s=30.0,
    invert_yaw=1,
    rate_hz=20.0,
    on_update=None,
):
    """
    Faehrt zum absoluten Gimbal-Yaw (SIYI-Telemetrie, normalisiert [-180, 180]).
    """
    abs_target = normalize_angle(abs_target_deg)
    t0 = time.monotonic()
    dt = 1.0 / rate_hz
    timed_out = False

    while True:
        current_abs = float(driver.current_attitude['yaw'])
        error = angle_diff(abs_target, current_abs)
        elapsed = time.monotonic() - t0

        if on_update is not None:
            on_update({
                'target': abs_target,
                'current': current_abs,
                'error': error,
                'elapsed': elapsed,
            })

        if abs(error) <= tolerance_deg:
            break

        if elapsed >= step_timeout_s:
            timed_out = True
            break

        direction = 1 if error > 0 else -1
        driver.set_gimbal_speed(direction * yaw_speed * invert_yaw, 0)
        time.sleep(dt)

    driver.set_gimbal_speed(0, 0)
    reached_abs = float(driver.current_attitude['yaw'])
    return {
        'target': abs_target,
        'reached': reached_abs,
        'error': angle_diff(abs_target, reached_abs),
        'duration_s': time.monotonic() - t0,
        'timed_out': timed_out,
    }


def move_to_relative_yaw(
    driver,
    home_abs,
    rel_target_deg,
    yaw_speed=15,
    tolerance_deg=2.0,
    step_timeout_s=30.0,
    invert_yaw=1,
    rate_hz=20.0,
    on_update=None,
):
    """Faehrt zum relativen Zielwinkel (nur fuer automatischen Sweep)."""
    abs_target = absolute_from_relative(home_abs, rel_target_deg)

    def wrapped(info):
        if on_update is None:
            return
        on_update({
            'rel_target': rel_target_deg,
            'rel_current': relative_yaw(home_abs, info['current']),
            'abs_target': info['target'],
            'abs_current': info['current'],
            'error': info['error'],
            'elapsed': info['elapsed'],
        })

    result = move_to_absolute_yaw(
        driver,
        abs_target,
        yaw_speed=yaw_speed,
        tolerance_deg=tolerance_deg,
        step_timeout_s=step_timeout_s,
        invert_yaw=invert_yaw,
        rate_hz=rate_hz,
        on_update=wrapped if on_update else None,
    )
    return {
        'rel_target': rel_target_deg,
        'rel_reached': relative_yaw(home_abs, result['reached']),
        'abs_target': result['target'],
        'abs_reached': result['reached'],
        'error': result['error'],
        'duration_s': result['duration_s'],
        'timed_out': result['timed_out'],
    }


def parse_yaw_list(text):
    """Komma-getrennte Werte parsen, z.B. '180,-180,0'."""
    parts = [p.strip() for p in str(text).split(',') if p.strip()]
    if not parts:
        raise ValueError('yaw_targets_deg darf nicht leer sein')
    return [float(p) for p in parts]


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('true', '1', 'yes')


def build_step_target(target_mode, values, step_index, step_origin_yaw):
    """
    Ziel-Yaw fuer einen Schritt berechnen.

    delta: values sind Drehwinkel relativ zum Yaw zu Schrittbeginn.
    absolute: values sind absolute Gimbal-Yaw-Winkel.
    """
    if step_index >= len(values):
        raise IndexError('step_index ausserhalb der Zielliste')
    if target_mode == 'delta':
        return normalize_angle(step_origin_yaw + values[step_index])
    if target_mode == 'absolute':
        return normalize_angle(values[step_index])
    raise ValueError(f'Unbekannter target_mode: {target_mode}')


def describe_sweep_plan(target_mode, values, home_yaw):
    """Menschenlesbare Vorschau aller Schritte."""
    lines = []
    origin = home_yaw
    for i, val in enumerate(values):
        target = build_step_target(target_mode, values, i, origin)
        if target_mode == 'delta':
            lines.append(
                f'  Schritt {i + 1}: von {origin:+.1f} deg, '
                f'Delta {val:+.1f} deg -> Ziel {target:+.1f} deg'
            )
            origin = target
        else:
            lines.append(
                f'  Schritt {i + 1}: absolutes Ziel {target:+.1f} deg'
            )
    return lines


# Vordefinierte Testszenarien fuer test_gimbal_sweep.py
SWEEP_SCENARIOS = {
    '1': {
        'name': 'Kurztest (+45 deg hin und zurueck)',
        'target_mode': 'delta',
        'yaw_targets_deg': '45,-45',
    },
    '2': {
        'name': 'Halbkreis (+180 deg und zurueck)',
        'target_mode': 'delta',
        'yaw_targets_deg': '180,-180',
    },
    '3': {
        'name': 'Rundum-Scan (+180, -180, zurueck auf Start)',
        'target_mode': 'delta',
        'yaw_targets_deg': '180,-180,0',
    },
}


class SweepState(Enum):
    WAIT_TELEMETRY = 'WAIT_TELEMETRY'
    CALIBRATE_DIR = 'CALIBRATE_DIR'
    GO_TARGET = 'GO_TARGET'
    SETTLING = 'SETTLING'
    DONE = 'DONE'
    FAILED = 'FAILED'


class GimbalSweepController:
    """Closed-Loop Gimbal-Sweep (ohne ROS-Abhaengigkeiten)."""

    def __init__(
        self,
        driver,
        yaw_targets,
        target_mode='delta',
        yaw_speed=20,
        tolerance_deg=2.0,
        step_timeout_s=30.0,
        settle_time_s=1.0,
        startup_delay_s=1.0,
        calibration_duration_s=0.6,
        invert_yaw='auto',
    ):
        self.driver = driver
        self.yaw_targets = list(yaw_targets)
        self.target_mode = target_mode
        self.yaw_speed_cmd = int(yaw_speed)
        self.tolerance_deg = float(tolerance_deg)
        self.step_timeout_s = float(step_timeout_s)
        self.settle_time_s = float(settle_time_s)
        self.startup_delay_s = float(startup_delay_s)
        self.calibration_duration_s = float(calibration_duration_s)

        invert_param = str(invert_yaw).lower()
        if invert_param in ('true', '1', 'yes'):
            self.invert_yaw = 1
            self.invert_yaw_auto = False
        elif invert_param in ('false', '0', 'no'):
            self.invert_yaw = -1
            self.invert_yaw_auto = False
        else:
            self.invert_yaw = 1
            self.invert_yaw_auto = True

        self.state = SweepState.WAIT_TELEMETRY
        self.state_enter_time = time.monotonic()
        self.step_index = 0
        self.current_target = None
        self.step_origin_yaw = 0.0
        self.home_yaw = 0.0
        self.start_yaw = 0.0
        self.command_yaw_speed = 0
        self.calibration_yaw_start = 0.0
        self.step_reports = []
        self._last_telemetry_yaw = None
        self._telemetry_alive = False

    def current_yaw(self):
        return float(self.driver.current_attitude['yaw'])

    def _set_state(self, new_state):
        self.state = new_state
        self.state_enter_time = time.monotonic()

    def _stop_gimbal(self):
        self.command_yaw_speed = 0
        self.driver.set_gimbal_speed(0, 0)

    def _start_next_target(self):
        if self.step_index >= len(self.yaw_targets):
            self._set_state(SweepState.DONE)
            self.current_target = None
            return False

        self.step_origin_yaw = self.current_yaw()
        self.current_target = build_step_target(
            self.target_mode,
            self.yaw_targets,
            self.step_index,
            self.step_origin_yaw,
        )
        self._set_state(SweepState.GO_TARGET)
        return True

    def _finish_step(self, reached_yaw, timed_out=False):
        target = self.current_target
        error = angle_diff(target, reached_yaw)
        duration = time.monotonic() - self.state_enter_time
        report = {
            'step': self.step_index + 1,
            'origin': self.step_origin_yaw,
            'delta': self.yaw_targets[self.step_index],
            'target': target,
            'reached': reached_yaw,
            'error': error,
            'duration_s': duration,
            'timed_out': timed_out,
        }
        self.step_reports.append(report)
        self._stop_gimbal()
        self.step_index += 1
        self._set_state(SweepState.SETTLING)
        return report

    def tick(self):
        """Ein Regeltakt. Gibt optional ein Status-Dict zurueck."""
        yaw = self.current_yaw()

        if self._last_telemetry_yaw is None:
            self._last_telemetry_yaw = yaw
        elif abs(yaw - self._last_telemetry_yaw) > 0.01:
            self._telemetry_alive = True
        self._last_telemetry_yaw = yaw

        status = {
            'state': self.state.value,
            'step': self.step_index + 1,
            'total_steps': len(self.yaw_targets),
            'target': self.current_target,
            'yaw': yaw,
            'error': angle_diff(self.current_target, yaw)
            if self.current_target is not None else 0.0,
            'cmd_speed': self.command_yaw_speed,
        }

        if self.state == SweepState.WAIT_TELEMETRY:
            if time.monotonic() - self.state_enter_time < self.startup_delay_s:
                return status
            self.home_yaw = yaw
            self.start_yaw = yaw
            self._set_state(SweepState.CALIBRATE_DIR)
            self.calibration_yaw_start = yaw
            self.command_yaw_speed = self.yaw_speed_cmd * self.invert_yaw
            self.driver.set_gimbal_speed(self.command_yaw_speed, 0)
            return status

        if self.state == SweepState.CALIBRATE_DIR:
            if time.monotonic() - self.state_enter_time < self.calibration_duration_s:
                return status
            self._stop_gimbal()
            delta = angle_diff(self.current_yaw(), self.calibration_yaw_start)
            if self.invert_yaw_auto:
                if delta > 0.5:
                    self.invert_yaw = 1
                elif delta < -0.5:
                    self.invert_yaw = -1
                else:
                    self.invert_yaw = 1
            self.step_index = 0
            self._start_next_target()
            return status

        if self.state == SweepState.GO_TARGET:
            target = self.current_target
            error = angle_diff(target, yaw)
            elapsed = time.monotonic() - self.state_enter_time

            if abs(error) <= self.tolerance_deg:
                self._finish_step(yaw, timed_out=False)
                return status

            if elapsed >= self.step_timeout_s:
                self._finish_step(yaw, timed_out=True)
                return status

            direction = 1 if error > 0 else -1
            self.command_yaw_speed = direction * self.yaw_speed_cmd * self.invert_yaw
            self.driver.set_gimbal_speed(self.command_yaw_speed, 0)
            status['error'] = error
            return status

        if self.state == SweepState.SETTLING:
            if time.monotonic() - self.state_enter_time >= self.settle_time_s:
                if not self._start_next_target():
                    self._stop_gimbal()
            return status

        if self.state == SweepState.DONE:
            self._stop_gimbal()
            return status

        return status

    def is_finished(self):
        return self.state in (SweepState.DONE, SweepState.FAILED)

    def stop(self):
        self._stop_gimbal()
