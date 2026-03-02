import cv2
import numpy as np
import requests
from ultralytics import YOLO
import time
import threading
from flask import Flask, Response, jsonify, send_from_directory
import os

# ================== ESP32 配置 ==================
esp32_main_ip = "10.143.39.206"     # 主 ESP32，负责 /capture
esp32_sub_ip  = "192.168.1.xxx"     # 副 ESP32，负责 /stream

url_main_capture = f"http://{esp32_main_ip}/capture"
url_main_action  = f"http://{esp32_main_ip}/action"
url_sub_stream   = f"http://{esp32_sub_ip}:81/stream"

# ================== YOLOv8 ==================
print("Loading YOLOv8 model...")
yolo_model = YOLO("yolov8n.pt")

# ================== 全局变量 ==================
latest_frame_main = None
latest_frame_sub  = None
last_vehicle_status = False

# ================== Flask 前端 ==================
current_dir = os.path.dirname(os.path.abspath(__file__))
server_folder = os.path.join(current_dir, "server")

app = Flask(__name__, static_folder=server_folder)

system_status = {
    "parking": [False, False],       # 两个摄像头
    "servo": ["Closed", "Closed"],
    "ssd1306": "placeholder.png"
}

# ================== 主视频合成流 ==================
def generate_frames():
    """合成主画面：左右拼接两个摄像头画面"""
    global latest_frame_main, latest_frame_sub

    while True:
        if latest_frame_main is not None and latest_frame_sub is not None:
            # 统一高度
            h = min(latest_frame_main.shape[0], latest_frame_sub.shape[0])
            frameA = cv2.resize(latest_frame_main, (h * 4 // 3, h))
            frameB = cv2.resize(latest_frame_sub,  (h * 4 // 3, h))

            combined = np.hstack([frameA, frameB])

            _, buffer = cv2.imencode(".jpg", combined)
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                   buffer.tobytes() + b"\r\n")
        else:
            time.sleep(0.1)

# ================== Flask API ==================
@app.route("/")
def index():
    return send_from_directory(server_folder, "index.html")

@app.route("/<path:file_path>")
def static_files(file_path):
    return send_from_directory(server_folder, file_path)

@app.route("/video")
def video():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/status")
def status():
    return jsonify(system_status)

def run_web_server():
    print("🌐 Web server running at http://localhost:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

# ================== Camera A：主 ESP32 ==================
def capture_main_cam():
    """使用 HTTP 单张 JPEG 抓取"""
    global latest_frame_main, system_status

    while True:
        try:
            resp = requests.get(url_main_capture, timeout=5)
            if resp.status_code == 200:
                img_array = np.frombuffer(resp.content, np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                latest_frame_main = frame
        except:
            pass
        time.sleep(0.03)

# ================== Camera B：副 ESP32 ==================
def capture_sub_cam():
    """使用 MJPEG 流抓取"""
    global latest_frame_sub

    cap = None
    while True:
        try:
            if cap is None:
                cap = cv2.VideoCapture(url_sub_stream)

            ret, frame = cap.read()
            if ret:
                latest_frame_sub = frame
            else:
                cap.release()
                cap = None
                time.sleep(1)
        except:
            time.sleep(1)

# ================== AI 处理主循环 ==================
def run_ai_loop():
    global last_vehicle_status
    global latest_frame_main
    global latest_frame_sub

    print("🚀 AI / Control Loop Running!")

    while True:
        if latest_frame_main is None or latest_frame_sub is None:
            time.sleep(0.05)
            continue

        # ================= YOLO: Camera A =================
        frameA = latest_frame_main.copy()
        resultsA = yolo_model(frameA, verbose=False)
        vehicleA = False

        for r in resultsA:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if cls in [2, 3, 5, 7] and conf > 0.5:
                    vehicleA = True
                    x1,y1,x2,y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frameA,(x1,y1),(x2,y2),(0,0,255),3)

        system_status["parking"][0] = vehicleA
        latest_frame_main = frameA

        # ================= YOLO: Camera B =================
        frameB = latest_frame_sub.copy()
        resultsB = yolo_model(frameB, verbose=False)
        vehicleB = False

        for r in resultsB:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if cls in [2, 3, 5, 7] and conf > 0.5:
                    vehicleB = True
                    x1,y1,x2,y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frameB,(x1,y1),(x2,y2),(0,255,0),3)

        system_status["parking"][1] = vehicleB
        latest_frame_sub = frameB

        # ================= 动作触发 (只用主摄) =================
        if vehicleA != last_vehicle_status:
            if vehicleA:
                print("🚨 Vehicle Detected (Cam A) → ESP32 ENTER")
                try:
                    requests.get(f"{url_main_action}?type=enter", timeout=0.3)
                    system_status["servo"][0] = "Open"
                except:
                    pass
            else:
                print("✅ Vehicle Cleared, gate closing")
                system_status["servo"][0] = "Closed"

            last_vehicle_status = vehicleA

        # ================= PC 显示 =================
        cv2.imshow("Camera A (Main)", frameA)
        cv2.imshow("Camera B (Sub)", frameB)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

# ================== 主程序 ==================
if __name__ == "__main__":
    # Flask
    threading.Thread(target=run_web_server, daemon=True).start()
    # 双摄采集线程
    threading.Thread(target=capture_main_cam, daemon=True).start()
    threading.Thread(target=capture_sub_cam,  daemon=True).start()

    # 主 AI 循环
    run_ai_loop()