#!/usr/bin/env python3
"""
Sprungantwort-Test: zeichnet Yaw nach Zielaenderung als CSV auf.

Hilft Telemetrie-Verzoegerung und Nachlauf zu diagnostizieren.
"""

import argparse
import csv
import sys
import time

from thermal_mapper.gimbal_control import (
    calibrate_yaw_direction,
    clamp_yaw,
    linear_yaw_error,
    move_to_absolute_yaw,
    read_yaw,
    scaled_yaw_speed,
)
from thermal_mapper.log_utils import ts, ts_iso
from thermal_mapper.siyi_driver import SiyiGimbalDriver


def record_sample(driver, phase, target, cmd_speed, t0):
    att, age_s = driver.get_attitude(fresh=True)
    mono = time.monotonic() - t0
    return {
        'wall_time': ts_iso(),
        't_s': f'{mono:.3f}',
        'phase': phase,
        'target_yaw': f'{target:.2f}',
        'yaw': f'{att["yaw"]:.2f}',
        'pitch': f'{att["pitch"]:.2f}',
        'roll': f'{att["roll"]:.2f}',
        'age_ms': f'{age_s * 1000.0:.1f}',
        'drain_n': driver.last_drain_count,
        'cmd_speed': cmd_speed,
    }


def write_row(writer, row):
    writer.writerow([
        row['wall_time'], row['t_s'], row['phase'], row['target_yaw'],
        row['yaw'], row['pitch'], row['roll'], row['age_ms'],
        row['drain_n'], row['cmd_speed'],
    ])


def parse_args():
    parser = argparse.ArgumentParser(
        description='Gimbal-Sprungantwort aufzeichnen (Yaw vs. Zeit)'
    )
    parser.add_argument(
        '--target',
        type=float,
        default=0.0,
        help='Absoluter Ziel-Yaw fuer den Sprung (Default: 0)',
    )
    parser.add_argument('--gimbal-ip', default='192.168.133.25')
    parser.add_argument('--local-ip', default='192.168.133.20')
    parser.add_argument('--yaw-speed', type=int, default=15)
    parser.add_argument('--tolerance-deg', type=float, default=2.0)
    parser.add_argument('--step-timeout-s', type=float, default=30.0)
    parser.add_argument('--baseline-s', type=float, default=2.0)
    parser.add_argument('--record-s', type=float, default=30.0)
    parser.add_argument('--sample-hz', type=float, default=10.0)
    parser.add_argument(
        '--csv',
        default='/tmp/gimbal_step_response.csv',
        help='Ausgabe-CSV',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    target = clamp_yaw(args.target)
    driver = None
    t0 = time.monotonic()
    dt = 1.0 / max(1.0, args.sample_hz)

    try:
        print(f'[{ts()}] Verbinde Gimbal {args.gimbal_ip} ...')
        driver = SiyiGimbalDriver(
            gimbal_ip=args.gimbal_ip,
            local_ip=args.local_ip,
        )
        driver.start_telemetry()
        time.sleep(1.0)

        invert_yaw = calibrate_yaw_direction(driver, args.yaw_speed)
        print(f'[{ts()}] invert_yaw={invert_yaw}')

        with open(args.csv, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'wall_time', 't_s', 'phase', 'target_yaw', 'yaw', 'pitch',
                'roll', 'age_ms', 'drain_n', 'cmd_speed',
            ])

            print(f'[{ts()}] Baseline {args.baseline_s:.1f}s ...')
            baseline_end = time.monotonic() + args.baseline_s
            while time.monotonic() < baseline_end:
                write_row(writer, record_sample(driver, 'baseline', target, 0, t0))
                csvfile.flush()
                time.sleep(dt)

            y0 = read_yaw(driver, fresh=True)
            print(
                f'[{ts()}] Sprung: {y0:+.1f} -> {target:+.1f} deg '
                f'(Aufzeichnung {args.record_s:.1f}s)'
            )

            move_t0 = time.monotonic()
            move_done = False
            cmd_speed = 0

            record_end = time.monotonic() + args.record_s
            while time.monotonic() < record_end:
                now = time.monotonic()

                if not move_done:
                    current = read_yaw(driver, fresh=True)
                    error = linear_yaw_error(target, current)
                    elapsed_move = now - move_t0

                    if abs(error) <= args.tolerance_deg:
                        driver.set_gimbal_speed(0, 0)
                        cmd_speed = 0
                        move_done = True
                        print(
                            f'[{ts()}] Ziel erreicht: {current:+.1f} deg '
                            f'(nach {elapsed_move:.1f}s), nehme Nachlauf auf ...'
                        )
                    elif elapsed_move >= args.step_timeout_s:
                        driver.set_gimbal_speed(0, 0)
                        cmd_speed = 0
                        move_done = True
                        print(f'[{ts()}] Move-Timeout nach {elapsed_move:.1f}s')
                    else:
                        direction = 1 if error > 0 else -1
                        cmd_speed = direction * scaled_yaw_speed(
                            abs(error), args.yaw_speed
                        ) * invert_yaw
                        driver.set_gimbal_speed(cmd_speed, 0)
                    phase = 'move' if not move_done else 'settle'
                else:
                    phase = 'settle'
                    cmd_speed = 0

                write_row(writer, record_sample(driver, phase, target, cmd_speed, t0))
                csvfile.flush()

                att, age_s = driver.get_attitude()
                sys.stdout.write(
                    f"\r[{ts()}] {phase:7s}  Ziel {target:+6.1f}  "
                    f"Yaw {att['yaw']:+7.1f}  age {age_s * 1000:5.0f}ms  "
                    f"drain {driver.last_drain_count:2d}  "
                )
                sys.stdout.flush()
                time.sleep(dt)

        print(f'\n[{ts()}] CSV gespeichert: {args.csv}')
        print(f'[{ts()}] Anzeigen: column -t -s, {args.csv} | less')

    except OSError as exc:
        print(f'[{ts()}] FEHLER: {exc}')
        sys.exit(1)
    except KeyboardInterrupt:
        print(f'\n[{ts()}] Abgebrochen.')
    finally:
        if driver is not None:
            driver.stop()


if __name__ == '__main__':
    main()
