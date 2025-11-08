from ultralytics import YOLO
import cv2
import serial
import time

# -------------------------------
# Configuration
# -------------------------------
MODEL_PATH = r"C:\Users\105-лаб-1\PycharmProjects\nonmilitary\runs\detect\train3\weights\best.pt"
CONFIDENCE_THRESHOLD = 0.60
UART_PORT = "COM3"           # or "/dev/ttyUSB0" on Linux
UART_BAUD = 115200
CAM_INDEX = 0
# -------------------------------

# Init model and camera
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAM_INDEX)
ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
time.sleep(2)  # allow Arduino reset

print("✅ Tracking started (sending servo angles via UART)...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Grayscale input
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    H, W = frame.shape[:2]

    results = model(gray3)
    detected = False

    for result in results:
        boxes = result.boxes.xywh
        confs = result.boxes.conf
        clss = result.boxes.cls

        for (x, y, w, h), conf, cls in zip(boxes, confs, clss):
            if int(cls) == 0 and conf >= CONFIDENCE_THRESHOLD:
                detected = True
                x, y = float(x), float(y)

                # --- Normalize and convert to servo degrees (0–180) ---
                x_norm = min(max(x / W, 0), 1)
                y_norm = min(max(y / H, 0), 1)

                x_deg = int(x_norm * 180)
                y_deg = int(y_norm * 180)

                # Send to Arduino
                ser.write(f"{x_deg},{y_deg}\n".encode())

                # Draw visualization
                cv2.circle(gray3, (int(x), int(y)), 5, (0, 255, 0), -1)
                cv2.putText(gray3,
                            f"X:{x_deg}° Y:{y_deg}°",
                            (int(x) - 40, int(y) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                print(f"Drone center: {x_deg:3d}°, {y_deg:3d}°")
                break

    if not detected:
        cv2.putText(gray3, "No drone detected",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Drone Tracking (UART)", gray3)
    if cv2.waitKey(1) == 27:  # ESC
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
