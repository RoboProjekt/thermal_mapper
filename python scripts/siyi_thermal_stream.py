import os
import cv2

# Schaltet die H.264 Time-Scale Warnungen von FFmpeg im Terminal stumm
os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "quiet"
# Erzwingt TCP für stabile Bilder
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

# Die IP der Kamera (aus deinem letzten erfolgreichen Test)
rtsp_url = "rtsp://192.168.133.25:8554/video1"

print(f"Versuche Verbindung herzustellen zu: {rtsp_url}")
cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("Fehler: Kamera ist nicht erreichbar.")
    exit()

print("Stream erfolgreich verbunden! Drücke 'q' im Bildfenster zum Beenden.")

# Erlaube OpenCV, das Fenster dynamisch anzupassen
cv2.namedWindow("SIYI ZT6 - Thermal View", cv2.WINDOW_NORMAL)
# Skaliere das Fenster auf die native Sensorauflösung (handlich)
cv2.resizeWindow("SIYI ZT6 - Thermal View", 640, 512)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Stream abgerissen.")
        break

    cv2.imshow("SIYI ZT6 - Thermal View", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()