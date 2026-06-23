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
        
        self.current_attitude = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
        self.running = True
        self.seq = 0  # Fortlaufende Sequenznummer für die Pakete

    def crc16_calculation(self, data):
        """ Offizieller SIYI CRC16 (XModem/CCITT) Algorithmus """
        crc = 0
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        # SIYI erwartet die CRC in Little-Endian (niedrigstwertiges Byte zuerst)
        return struct.pack('<H', crc)

    def send_command(self, cmd_id, data=b''):
        """ Baut das Paket inkl. Header, Payload und berechneter Checksumme zusammen """
        # STX (0x55 0x66) + CTRL (0x01 = Need no ACK)
        header = b'\x55\x66\x01' 
        data_len = struct.pack('<H', len(data))
        seq_bytes = struct.pack('<H', self.seq)
        
        # Sequenznummer für das nächste Paket erhöhen (Wrap-around bei 65535)
        self.seq = (self.seq + 1) & 0xFFFF 
        
        # Nachricht ohne CRC zusammenbauen
        msg_without_crc = header + data_len + seq_bytes + bytes([cmd_id]) + data
        
        # CRC berechnen und anhängen
        msg_with_crc = msg_without_crc + self.crc16_calculation(msg_without_crc)
        
        # Abfeuern!
        self.sock.sendto(msg_with_crc, self.gimbal_addr)

    def request_attitude(self):
        """ Sendet Command 0x0D (Acquire Gimbal Attitude) """
        self.send_command(0x0D)

    def parse_response(self, data):
        """ Parsen der Antwort von Command 0x0D """
        # Ein gültiges Attitude-Paket (0x0D) hat in der Regel eine spezifische Länge
        # Index 7 ist die CMD_ID im Antwortpaket von SIYI
        if len(data) >= 14 and data[7] == 0x0D:
            # Daten fangen ab Index 8 an: Yaw, Pitch, Roll (Int16, Little-Endian)
            yaw, pitch, roll = struct.unpack('<hhh', data[8:14])
            
            # Umrechnung in echte Grad (SIYI sendet Zehntelgrad)
            self.current_attitude["yaw"] = yaw / 10.0
            self.current_attitude["pitch"] = pitch / 10.0
            self.current_attitude["roll"] = roll / 10.0
            return self.current_attitude
        return None

    def set_gimbal_speed(self, yaw_speed, pitch_speed):
        """ Sendet Command 0x07 (Gimbal Speed) - Wertebereich -100 bis 100 """
        data = struct.pack('bb', yaw_speed, pitch_speed)
        self.send_command(0x07, data)

    def telemetry_loop(self):
        while self.running:
            self.request_attitude()
            try:
                data, _ = self.sock.recvfrom(1024)
                self.parse_response(data)
            except socket.timeout:
                pass
            time.sleep(0.05) # 20Hz Update Rate
            
    
    def set_zoom(self, zoom_dir):
        """ Sendet Command 0x05 (Manual Zoom & Auto Focus) 
            1 = Zoom In, -1 = Zoom Out, 0 = Stop """
        data = struct.pack('b', zoom_dir)
        self.send_command(0x05, data)
        

if __name__ == "__main__":
    import sys
    import tty
    import termios
    import select

    print("Starte SIYI Gimbal Treiber...")
    driver = SiyiGimbalDriver(gimbal_ip="192.168.133.25", local_ip="192.168.133.20")
    
    telemetry_thread = threading.Thread(target=driver.telemetry_loop)
    telemetry_thread.start()
    
    print("\n" + "="*45)
    print("GIMBAL STEUERUNG AKTIV (Dauerfeuer-Modus)")
    print("W / S : Nicken (Pitch up/down)")
    print("A / D : Gieren (Yaw left/right)")
    print("Q / E : Zoom (Out / In)")
    print("X     : Alles stoppen (0, 0)")
    print("STRG+C: Beenden")
    print("="*45 + "\n")

    def is_data():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    old_settings = termios.tcgetattr(sys.stdin)
    
    current_yaw_speed = 0
    current_pitch_speed = 0
    current_zoom_dir = 0
    speed = 20 

    try:
        tty.setcbreak(sys.stdin.fileno())
        
        while True:
            if is_data():
                key = sys.stdin.read(1).lower()
                
                if key == '\x03': # STRG+C fängt das Programm sauber ab
                    break
                elif key == 'w':
                    current_pitch_speed = speed; current_yaw_speed = 0; current_zoom_dir = 0
                elif key == 's':
                    current_pitch_speed = -speed; current_yaw_speed = 0; current_zoom_dir = 0
                elif key == 'a':
                    current_yaw_speed = -speed; current_pitch_speed = 0; current_zoom_dir = 0
                elif key == 'd':
                    current_yaw_speed = speed; current_pitch_speed = 0; current_zoom_dir = 0
                elif key == 'e':
                    current_yaw_speed = 0; current_pitch_speed = 0; current_zoom_dir = 1  # Zoom In
                elif key == 'q':
                    current_yaw_speed = 0; current_pitch_speed = 0; current_zoom_dir = -1 # Zoom Out
                elif key == 'x':
                    current_yaw_speed = 0; current_pitch_speed = 0; current_zoom_dir = 0  # Stop

            # Sende Ausrichtung
            driver.set_gimbal_speed(current_yaw_speed, current_pitch_speed)
            # Sende Zoom-Befehl
            driver.set_zoom(current_zoom_dir)
            
            sys.stdout.write(f"\rPitch Cmd: {current_pitch_speed:3d} | Yaw: {current_yaw_speed:3d} | Zoom: {current_zoom_dir:2d} "
                             f"|| Tel -> Pitch: {driver.current_attitude['pitch']:6.1f}° | Yaw: {driver.current_attitude['yaw']:6.1f}°")
            sys.stdout.flush()
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nBeende Treiber...")
        driver.set_gimbal_speed(0, 0)
        driver.set_zoom(0)
        driver.running = False
        telemetry_thread.join()
        driver.sock.close()
        print("Erfolgreich beendet.")