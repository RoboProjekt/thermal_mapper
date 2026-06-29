#!/usr/bin/env python3
"""Interaktiver Test fuer SiyiGimbalDriver (Telemetrie + set_gimbal_speed)."""

import argparse
import sys
import time

from thermal_mapper.siyi_driver import SiyiGimbalDriver


def print_menu():
    print()
    print("=" * 50)
    print("SIYI Gimbal Treiber-Test")
    print("=" * 50)
    print("  1  Yaw rechts  (+yaw_speed, dann Stop)")
    print("  2  Yaw links   (-yaw_speed, dann Stop)")
    print("  3  Telemetrie   (nur lesen, kein Motorbefehl)")
    print("  q  Beenden")
    print("=" * 50)


def format_attitude(att):
    return (
        f"Yaw={att['yaw']:7.1f}  "
        f"Pitch={att['pitch']:7.1f}  "
        f"Roll={att['roll']:7.1f}"
    )


def wait_telemetry(driver, seconds):
    """Kurz warten bis erste Telemetrie-Pakete ankommen."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        att = driver.current_attitude
        if any(abs(v) > 0.01 for v in att.values()):
            return True
        time.sleep(0.1)
    return True


def scenario_yaw(driver, yaw_speed, duration_s, label):
    """Gimbal in Yaw-Richtung drehen und Delta ausgeben."""
    att0 = dict(driver.current_attitude)
    print(f"\n--- {label} ---")
    print(f"Start:  {format_attitude(att0)}")
    print(f"Befehl: set_gimbal_speed({yaw_speed}, 0) fuer {duration_s:.1f} s")

    driver.set_gimbal_speed(yaw_speed, 0)
    t_end = time.time() + duration_s
    while time.time() < t_end:
        sys.stdout.write(f"\r{format_attitude(driver.current_attitude)}")
        sys.stdout.flush()
        time.sleep(0.1)

    driver.set_gimbal_speed(0, 0)
    time.sleep(0.3)
    att1 = dict(driver.current_attitude)

    print()
    print(f"Ende:   {format_attitude(att1)}")
    dyaw = att1["yaw"] - att0["yaw"]
    print(f"Delta Yaw: {dyaw:+.1f} deg")

    if abs(dyaw) < 0.5:
        print("HINWEIS: Kaum Bewegung – Gimbal erreichbar? Speed erhoehen?")
    else:
        print("OK: Yaw hat sich geaendert.")
    print()


def scenario_telemetry(driver, duration_s):
    """Nur Telemetrie anzeigen, kein Motorbefehl."""
    print(f"\n--- Telemetrie ({duration_s:.0f} s, kein Motorbefehl) ---")
    print("STRG+C zum Abbrechen\n")

    t_end = time.time() + duration_s
    samples = 0
    while time.time() < t_end:
        sys.stdout.write(f"\r{format_attitude(driver.current_attitude)}")
        sys.stdout.flush()
        samples += 1
        time.sleep(0.2)

    print(f"\n\nOK: {samples} Anzeige-Updates empfangen.")
    print()


def run_scenario(choice, driver, yaw_speed, duration_s, telemetry_duration_s):
    if choice == "1":
        scenario_yaw(driver, yaw_speed, duration_s, "Yaw rechts")
    elif choice == "2":
        scenario_yaw(driver, -yaw_speed, duration_s, "Yaw links")
    elif choice == "3":
        scenario_telemetry(driver, telemetry_duration_s)
    else:
        print(f"Unbekannte Auswahl: {choice}")
        return False
    return True


def parse_args():
    parser = argparse.ArgumentParser(
        description="SIYI Gimbal Treiber-Test (Szenario 1/2/3)"
    )
    parser.add_argument(
        "scenario",
        nargs="?",
        choices=["1", "2", "3"],
        help="Direkt starten ohne Menue (1=rechts, 2=links, 3=Telemetrie)",
    )
    parser.add_argument("--gimbal-ip", default="192.168.133.25")
    parser.add_argument("--local-ip", default="192.168.133.20")
    parser.add_argument(
        "--yaw-speed",
        type=int,
        default=15,
        help="Geschwindigkeit fuer Szenario 1/2 (Default: 15)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Dauer der Yaw-Bewegung in Sekunden (Default: 2.0)",
    )
    parser.add_argument(
        "--telemetry-duration",
        type=float,
        default=5.0,
        help="Dauer Szenario 3 in Sekunden (Default: 5.0)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    driver = None

    try:
        print(f"Verbinde Gimbal {args.gimbal_ip} (lokal {args.local_ip}) ...")
        driver = SiyiGimbalDriver(
            gimbal_ip=args.gimbal_ip,
            local_ip=args.local_ip,
        )
        driver.start_telemetry()
        wait_telemetry(driver, 1.5)
        print(f"Telemetrie: {format_attitude(driver.current_attitude)}")

        if args.scenario:
            run_scenario(
                args.scenario,
                driver,
                args.yaw_speed,
                args.duration,
                args.telemetry_duration,
            )
            return

        while True:
            print_menu()
            choice = input("Auswahl [1/2/3/q]: ").strip().lower()
            if choice in ("q", "quit", "exit"):
                break
            if choice not in ("1", "2", "3"):
                print("Bitte 1, 2, 3 oder q eingeben.")
                continue
            run_scenario(
                choice,
                driver,
                args.yaw_speed,
                args.duration,
                args.telemetry_duration,
            )

    except OSError as exc:
        print(f"\nFEHLER: UDP-Verbindung fehlgeschlagen: {exc}")
        print("Pruefe Kabel, IP 192.168.133.25 und lokales Interface 192.168.133.20.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nAbgebrochen.")
    finally:
        if driver is not None:
            print("Sende Stop und schliesse Treiber ...")
            driver.stop()
            print("Fertig.")


if __name__ == "__main__":
    main()
