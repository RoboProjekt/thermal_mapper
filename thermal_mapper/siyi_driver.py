"""SIYI ZT6 Gimbal UDP-Treiber (CRC16, Attitude-Telemetrie)."""

import socket
import struct
import threading
import time


class SiyiGimbalDriver:
    def __init__(self, gimbal_ip="192.168.133.25", local_ip="192.168.133.20", port=37260):
        self.gimbal_addr = (gimbal_ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Explizites Binding an das USB-Ethernet Interface
        self.sock.bind((local_ip, 0))
        self.sock.settimeout(0.1)

        self._sock_lock = threading.Lock()
        self._att_lock = threading.Lock()
        self.current_attitude = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
        self.attitude_updated_mono = 0.0
        self.last_drain_count = 0
        self.running = True
        self.seq = 0

    def crc16_calculation(self, data):
        """Offizieller SIYI CRC16 (XModem/CCITT) Algorithmus."""
        crc = 0
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return struct.pack('<H', crc)

    def send_command(self, cmd_id, data=b''):
        header = b'\x55\x66\x01'
        data_len = struct.pack('<H', len(data))
        seq_bytes = struct.pack('<H', self.seq)
        self.seq = (self.seq + 1) & 0xFFFF

        msg_without_crc = header + data_len + seq_bytes + bytes([cmd_id]) + data
        msg_with_crc = msg_without_crc + self.crc16_calculation(msg_without_crc)
        self.sock.sendto(msg_with_crc, self.gimbal_addr)

    def request_attitude(self):
        """Sendet Command 0x0D (Acquire Gimbal Attitude)."""
        self.send_command(0x0D)

    def parse_response(self, data):
        """Parsen der Antwort von Command 0x0D."""
        if len(data) >= 14 and data[7] == 0x0D:
            yaw, pitch, roll = struct.unpack('<hhh', data[8:14])
            with self._att_lock:
                self.current_attitude["yaw"] = yaw / 10.0
                self.current_attitude["pitch"] = pitch / 10.0
                self.current_attitude["roll"] = roll / 10.0
                self.attitude_updated_mono = time.monotonic()
            return True
        return False

    def _drain_recv(self, timeout_s=0.08):
        """
        Empfangspuffer leeren und neueste 0x0D-Antwort behalten.
        Verhindert veraltete Yaw-Werte aus UDP-Backlog.
        """
        end = time.monotonic() + timeout_s
        parsed = 0
        while time.monotonic() < end:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            try:
                self.sock.settimeout(remaining)
                data, _ = self.sock.recvfrom(1024)
                if self.parse_response(data):
                    parsed += 1
            except socket.timeout:
                break
        self.sock.settimeout(0.1)
        self.last_drain_count = parsed
        return parsed

    def refresh_attitude(self, drain_timeout_s=0.08):
        """Attitude anfordern und alle wartenden Antworten einlesen."""
        with self._sock_lock:
            self.request_attitude()
            self._drain_recv(drain_timeout_s)

    def get_attitude(self, fresh=False):
        """
        Aktuelle Attitude und Alter in Sekunden.
        fresh=True: vor dem Lesen neu pollen (fuer Status-Anzeige).
        """
        if fresh:
            self.refresh_attitude()
        with self._att_lock:
            age = time.monotonic() - self.attitude_updated_mono
            return dict(self.current_attitude), age

    def set_gimbal_speed(self, yaw_speed, pitch_speed):
        """Sendet Command 0x07 (Gimbal Speed), Wertebereich -100 bis 100."""
        yaw_speed = max(-100, min(100, int(yaw_speed)))
        pitch_speed = max(-100, min(100, int(pitch_speed)))
        data = struct.pack('bb', yaw_speed, pitch_speed)
        with self._sock_lock:
            self.send_command(0x07, data)

    def telemetry_loop(self):
        while self.running:
            try:
                self.refresh_attitude()
            except OSError:
                pass
            time.sleep(0.05)  # 20 Hz

    def start_telemetry(self):
        """Startet den Telemetrie-Thread."""
        thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        try:
            self.set_gimbal_speed(0, 0)
        except OSError:
            pass
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
