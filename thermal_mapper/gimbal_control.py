"""Closed-Loop Gimbal-Yaw Steuerung (Geschwindigkeit 0x07)."""

import time

# SIYI ZT6: Yaw ist linear, kein 360-Grad-Wrap in der Telemetrie
SIYI_YAW_MIN = -270.0
SIYI_YAW_MAX = 270.0

# Ab dieser Fehlergroesse wird die Geschwindigkeit reduziert (weniger Overshoot)
SLOW_ZONE_DEG = 25.0
MIN_YAW_SPEED = 5


def clamp_yaw(deg):
    """Zielwinkel auf den SIYI-Bereich begrenzen."""
    return max(SIYI_YAW_MIN, min(SIYI_YAW_MAX, float(deg)))


def linear_yaw_error(target_deg, current_deg):
    """Linearer Fehler target - current (kein Umweg ueber 360 Grad)."""
    return float(target_deg) - float(current_deg)


def read_yaw(driver, fresh=False):
    att, _age = driver.get_attitude(fresh=fresh)
    return float(att['yaw'])


def calibrate_yaw_direction(driver, yaw_speed, duration_s=0.5, tick_callback=None):
    """Kurzer Impuls: liefert +1 oder -1 fuer set_gimbal_speed Vorzeichen."""
    start = read_yaw(driver, fresh=True)
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
    delta = linear_yaw_error(read_yaw(driver, fresh=True), start)
    if delta > 0.5:
        return 1
    if delta < -0.5:
        return -1
    return 1


def scaled_yaw_speed(abs_error, yaw_speed):
    """Volle Geschwindigkeit weit weg, langsamere Annaeherung nahe am Ziel."""
    if abs_error >= SLOW_ZONE_DEG:
        return yaw_speed
    scale = abs_error / SLOW_ZONE_DEG
    return max(MIN_YAW_SPEED, int(yaw_speed * scale))


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
    """Faehrt zum absoluten Gimbal-Yaw (SIYI-Telemetrie, linear)."""
    abs_target = clamp_yaw(abs_target_deg)
    t0 = time.monotonic()
    dt = 1.0 / rate_hz
    timed_out = False

    while True:
        if tick_callback is not None:
            tick_callback()

        current_abs = read_yaw(driver, fresh=True)
        att, age_s = driver.get_attitude()
        error = linear_yaw_error(abs_target, current_abs)
        elapsed = time.monotonic() - t0

        if on_update is not None:
            on_update({
                'target': abs_target,
                'current': current_abs,
                'error': error,
                'elapsed': elapsed,
                'age_s': age_s,
            })

        if abs(error) <= tolerance_deg:
            break

        if elapsed >= step_timeout_s:
            timed_out = True
            break

        direction = 1 if error > 0 else -1
        cmd = direction * scaled_yaw_speed(abs(error), yaw_speed) * invert_yaw
        driver.set_gimbal_speed(cmd, 0)
        time.sleep(dt)

    driver.set_gimbal_speed(0, 0)
    if tick_callback is not None:
        tick_callback()
    reached_abs = read_yaw(driver, fresh=True)
    return {
        'target': abs_target,
        'reached': reached_abs,
        'error': linear_yaw_error(abs_target, reached_abs),
        'duration_s': time.monotonic() - t0,
        'timed_out': timed_out,
    }
