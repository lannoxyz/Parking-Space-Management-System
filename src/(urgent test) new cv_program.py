import cv2
import numpy as np
import requests
from ultralytics import YOLO
import time
import threading
from flask import Flask, Response, jsonify, send_from_directory

# ================= CONFIGURATION =================
ESP32_IP = "192.168.0.25"

URL_CAPTURE = f"http://{ESP32_IP}/capture"
URL_ACTION  = f"http://{ESP32_IP}/action"

print("Loading YOLOv8 model...")
model = YOLO("yolov8n.pt")

last_vehicle_status = False
latest_frame = None

# ================= WEB SERVER =================
app = Flask(__name__, static_folder="server")

system_status = {
    "parking": [False, False, False, False, False],
    "servo": ["Closed", "Closed"],
    "ssd1306": "placeholder.png"
}

def generate_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            ret, buffer = cv2.imencode('.jpg', latest_frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)

@app.route("/")
def index():
    return send_from_directory("server", "index.html")

@app.route("/<path:path>")
def static_files(path):
    return send_from_directory("server", path)

@app.route("/video")
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    return jsonify(system_status)

def run_web():
    print("🌐 Web server running on http://localhost:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

# ================= AI LOOP =================
def ai_loop():
    global last_vehicle_status
    global latest_frame

    print(f"🚀 System started! Connecting to: {ESP32_IP}")
    print("Press 'q' to exit")

    while True:
        try:
            response = requests.get(URL_CAPTURE, timeout=5)

            if response.status_code == 200:
                img_array = np.array(bytearray(response.content), dtype=np.uint8)
                frame = cv2.imdecode(img_array, -1)

                if frame is None:
                    continue

                results = model(frame, verbose=False)
                vehicle_detected = False

                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])

                        # COCO IDs
                        if cls_id in [2, 3, 5, 7] and conf > 0.5:
                            vehicle_detected = True

                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            label = f"{model.names[cls_id]} {conf:.2f}"

                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                            cv2.putText(frame, label,
                                        (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5,
                                        (0, 0, 255),
                                        2)

                # 更新最新帧给网页
                latest_frame = frame.copy()

                # 状态变化才发送指令
                if vehicle_detected != last_vehicle_status:
                    if vehicle_detected:
                        print("🚨 Vehicle Detected -> Turn Light ON!")
                        try:
                            requests.get(f"{URL_ACTION}?val=1", timeout=0.5)
                        except:
                            pass
                    else:
                        print("✅ Vehicle Cleared -> Turn Light OFF")
                        try:
                            requests.get(f"{URL_ACTION}?val=0", timeout=0.5)
                        except:
                            pass

                    last_vehicle_status = vehicle_detected

                # 更新 mock parking 状态（演示用）
                system_status["parking"][0] = vehicle_detected

                cv2.imshow("ESP32 AI Camera", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            else:
                print("❌ Failed to fetch image (HTTP Error)")

        except Exception as e:
            print(f"Connection error (Retrying...): {e}")
            time.sleep(1)

    cv2.destroyAllWindows()

# ================= MAIN =================
if __name__ == "__main__":
    threading.Thread(target=run_web, daemon=True).start()
    ai_loop()