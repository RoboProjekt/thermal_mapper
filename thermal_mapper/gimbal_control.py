"""Closed-Loop Gimbal-Yaw Steuerung (Geschwindigkeit 0x07)."""

import time


def normalize_angle(deg):
    """Winkel auf [-180, 180] Grad normalisieren."""
    return (deg + 180.0) % 360.0 - 180.0


def angle_diff(target_deg, current_deg):
    """Kuerzester Winkel target - current in Grad."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def calibrate_yaw_direction(driver, yaw_speed, duration_s=0.5, tick_callback=None):
    """Kurzer Impuls: liefert +1 oder -1 fuer set_gimbal_speed Vorzeichen."""
    start = float(driver.current_attitude['yaw'])
    driver.set_gimbal_speed(yaw_speed, 0)
    t_end = time.monotonic() + duration_s
    while time.monotonic() < t_end:
        if tick_callback is not None:
            tick_callback()
        time.sleep(0.05)
    driver.set_gimbal_speed(0, 0)
    time.sleep(0.2)
    if tick_callback is not None:
        tick_callback()
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
    tick_callback=None,
):
    """Faehrt zum absoluten Gimbal-Yaw (SIYI-Telemetrie)."""
    abs_target = normalize_angle(abs_target_deg)
    t0 = time.monotonic()
    dt = 1.0 / rate_hz
    timed_out = False

    while True:
        if tick_callback is not None:
            tick_callback()

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
    if tick_callback is not None:
        tick_callback()
    reached_abs = float(driver.current_attitude['yaw'])
    return {
        'target': abs_target,
        'reached': reached_abs,
        'error': angle_diff(abs_target, reached_abs),
        'duration_s': time.monotonic() - t0,
        'timed_out': timed_out,
    }
