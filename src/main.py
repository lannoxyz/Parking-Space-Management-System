import cv2
import threading
import time
import requests
import numpy as np
from flask import Flask, Response, jsonify

# ======================================
# ESP32 Camera Endpoints
# ======================================
CAM1_URL = "http://10.143.39.206/capture"   # Entrance (AI)
CAM2_URL = "http://10.143.39.6/capture"   # Exit

# ======================================
# Shared Frame Buffers
# ======================================
latest_cam1 = None  # entrance
latest_cam2 = None  # exit

# ======================================
# YOLO Placeholder (you may plug in actual)
# ======================================
def run_yolo(frame):
    # ---- placeholder ----
    # 返回原图即可（方便你后续接 YOLOv5、YOLOv8）
    return frame

# ======================================
# Frame Grabber Thread
# ======================================
def grab_thread(url, cam_id):
    global latest_cam1, latest_cam2

    while True:
        try:
            r = requests.get(url, timeout=1)
            if r.status_code == 200:
                jpg = np.frombuffer(r.content, dtype=np.uint8)
                frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)

                # AI detection for CAM1
                if cam_id == 1:
                    frame = run_yolo(frame)
                    latest_cam1 = frame
                else:
                    latest_cam2 = frame
        except Exception as e:
            print(f"[Camera {cam_id}] Error:", e)

        time.sleep(0.03)

# ======================================
# MJPEG Stream Generator
# ======================================
def mjpeg_stream(camera_id):
    global latest_cam1, latest_cam2
    while True:
        frame = latest_cam1 if camera_id == 1 else latest_cam2
        if frame is None:
            continue
        _, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            frame_bytes +
            b"\r\n"
        )

# ======================================
# Flask Server
# ======================================
app = Flask(__name__)

@app.route("/video")
def video1():
    return Response(mjpeg_stream(1), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video2")
def video2():
    return Response(mjpeg_stream(2), mimetype="multipart/x-mixed-replace; boundary=frame")

# ======================================
# Status JSON (给你的 JS 用)
# ======================================
@app.route("/status")
def status():
    # 你现在不需要车位状态/伺服器，我先用默认值保持前端不报错
    return jsonify({
        "parking": [False, False, False, False], 
        "servo": ["Closed", "Closed"]
    })

# ======================================
# Start Threads + Server
# ======================================
if __name__ == "__main__":
    print("Starting camera threads...")
    t1 = threading.Thread(target=grab_thread, args=(CAM1_URL, 1), daemon=True)
    t2 = threading.Thread(target=grab_thread, args=(CAM2_URL, 2), daemon=True)

    t1.start()
    t2.start()

    print("Starting Flask server at http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True)