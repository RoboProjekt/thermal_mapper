#!/usr/bin/env python3
"""
Interaktiver Gimbal-Test mit absoluten Yaw-Winkeln (SIYI-Telemetrie).

Eingabe = absoluter Zielwinkel in Grad, z.B. 0, 90, -180.
Die Startposition beim Verbinden ist dafuer nicht relevant.
"""

import argparse
import sys
import time

from thermal_mapper.gimbal_sweep_controller import (
    calibrate_yaw_direction,
    move_to_absolute_yaw,
    normalize_angle,
)
from thermal_mapper.siyi_driver import SiyiGimbalDriver


def wait_telemetry(driver, seconds=1.0):
    time.sleep(seconds)
    return float(driver.current_attitude['yaw'])


def print_status(current_yaw):
    print(f'  Aktueller Yaw (absolut): {current_yaw:+7.1f} deg')


def print_help():
    print()
    print('Befehle:')
    print('  <zahl>   Absoluter Ziel-Yaw in Grad, z.B. 0, 90, -180, 180')
    print('  s        Nur Status anzeigen')
    print('  q        Beenden')
    print()


def live_update_factory():
    def on_update(info):
        sys.stdout.write(
            f"\r  -> Ziel {info['target']:+6.1f} deg  "
            f"Ist {info['current']:+6.1f} deg  "
            f"Fehler {info['error']:+5.1f} deg  "
            f"{info['elapsed']:4.1f}s  "
        )
        sys.stdout.flush()
    return on_update


def run_move(driver, abs_target, yaw_speed, tolerance, timeout, invert_yaw):
    abs_target = normalize_angle(abs_target)
    print(f'\nFahre zu absolut {abs_target:+.1f} deg ...')
    result = move_to_absolute_yaw(
        driver,
        abs_target,
        yaw_speed=yaw_speed,
        tolerance_deg=tolerance,
        step_timeout_s=timeout,
        invert_yaw=invert_yaw,
        on_update=live_update_factory(),
    )
    print()
    flag = 'TIMEOUT' if result['timed_out'] else 'OK'
    print(
        f'  [{flag}] Ziel {result["target"]:+.1f} deg  '
        f'erreicht {result["reached"]:+.1f} deg  '
        f'Fehler {result["error"]:+.1f} deg  '
        f'({result["duration_s"]:.1f}s)'
    )
    return result


def parse_args():
    parser = argparse.ArgumentParser(
        description='Gimbal auf absolute Yaw-Winkel fahren (SIYI-Telemetrie)'
    )
    parser.add_argument('--gimbal-ip', default='192.168.133.25')
    parser.add_argument('--local-ip', default='192.168.133.20')
    parser.add_argument('--yaw-speed', type=int, default=15)
    parser.add_argument('--tolerance-deg', type=float, default=2.0)
    parser.add_argument('--step-timeout-s', type=float, default=30.0)
    parser.add_argument(
        '--target',
        type=float,
        default=None,
        help='Optional: direkt zu diesem absoluten Winkel fahren',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    driver = None

    try:
        print(f'Verbinde Gimbal {args.gimbal_ip} (lokal {args.local_ip}) ...')
        driver = SiyiGimbalDriver(
            gimbal_ip=args.gimbal_ip,
            local_ip=args.local_ip,
        )
        driver.start_telemetry()
        wait_telemetry(driver)

        invert_yaw = calibrate_yaw_direction(driver, args.yaw_speed)

        print()
        print('=' * 58)
        print('Steuerung mit absoluten Yaw-Winkeln (SIYI-Telemetrie)')
        print('Eingabe = Zielwinkel in Grad, unabhaengig von der Startposition.')
        print('=' * 58)
        print_status(float(driver.current_attitude['yaw']))
        print_help()

        if args.target is not None:
            run_move(
                driver, args.target,
                args.yaw_speed, args.tolerance_deg,
                args.step_timeout_s, invert_yaw,
            )

        while True:
            print()
            print_status(float(driver.current_attitude['yaw']))
            try:
                raw = input('Ziel-Yaw absolut (Grad): ').strip()
            except EOFError:
                break

            cmd = raw.lower()
            if cmd in ('q', 'quit', 'exit'):
                break
            if cmd in ('', 's', 'status'):
                continue

            try:
                abs_target = float(raw)
            except ValueError:
                print('Ungueltige Eingabe. Zahl, s oder q.')
                continue

            run_move(
                driver, abs_target,
                args.yaw_speed, args.tolerance_deg,
                args.step_timeout_s, invert_yaw,
            )

    except OSError as exc:
        print(f'\nFEHLER: UDP-Verbindung fehlgeschlagen: {exc}')
        sys.exit(1)
    except KeyboardInterrupt:
        print('\n\nAbgebrochen.')
    finally:
        if driver is not None:
            print('Sende Stop ...')
            driver.stop()
            print('Fertig.')


if __name__ == '__main__':
    main()
