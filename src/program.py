import cv2
import threading
import time
import requests
import numpy as np
from flask import Flask, Response, jsonify, render_template
from datetime import datetime

app = Flask(__name__, template_folder='templates', static_folder='static')

MAIN_ESP_URL = "http://192.168.100.68"   # Camera ESP32
SUB_ESP_URL  = "http://192.168.100.86"  # Control ESP32

lock = threading.Lock()
raw_frame       = None
processed_frame = None

system_state = {
    "parking":    [False, False, False, False],
    "servo":      ["Closed", "Closed"],
    "freq":       [0, 0, 0, 0],
    "last_seen":  "Never",
    "cam_online": False,
    "sub_online": False,
}

# ============================================================
# YOLO / CV Pipeline  ← FILL IN YOUR MODEL HERE
# ============================================================
def run_cv_pipeline(frame):
    annotated = frame.copy()
    # TODO: Replace with your YOLO inference
    # results = model.predict(frame, conf=0.5)
    # annotated = results[0].plot()
    cv2.putText(annotated, "CV/YOLO Placeholder",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 120), 2)
    cv2.putText(annotated, datetime.now().strftime("%H:%M:%S"),
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
    return annotated, None   # (annotated_frame, "car_enter"|"car_exit"|None)

# ============================================================
# Gate Trigger
# ============================================================
last_gate_trigger = 0
GATE_COOLDOWN = 7.0

def trigger_gate(gate_type):
    global last_gate_trigger
    now = time.time()
    if now - last_gate_trigger < GATE_COOLDOWN:
        return
    last_gate_trigger = now
    def _send():
        try:
            requests.get(f"{SUB_ESP_URL}/action?servo={gate_type}", timeout=2)
            with lock:
                system_state["servo"][0 if gate_type == "in" else 1] = "Open"
        except Exception as e:
            print(f"⚠️ Gate trigger failed: {e}")
    threading.Thread(target=_send, daemon=True).start()

# ============================================================
# Threads
# ============================================================
def camera_grabber():
    global raw_frame, processed_frame
    fails = 0
    while True:
        try:
            r = requests.get(f"{MAIN_ESP_URL}/capture", timeout=1.5)
            if r.status_code == 200:
                arr   = np.frombuffer(r.content, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None: raise ValueError("Bad JPEG")
                annotated, detection = run_cv_pipeline(frame)
                if detection == "car_enter": trigger_gate("in")
                elif detection == "car_exit": trigger_gate("out")
                with lock:
                    raw_frame       = frame
                    processed_frame = annotated
                    system_state["cam_online"] = True
                    system_state["last_seen"]  = datetime.now().strftime("%H:%M:%S")
                fails = 0
        except Exception as e:
            fails += 1
            with lock:
                system_state["cam_online"] = False
            if fails % 10 == 1:
                print(f"[CAM] Offline ({fails}): {e}")
        time.sleep(0.05)

def sub_esp_poller():
    while True:
        try:
            r = requests.get(f"{SUB_ESP_URL}/status", timeout=1.5)
            if r.status_code == 200:
                data = r.json()
                with lock:
                    system_state["parking"]    = data.get("parking", [False]*4)
                    system_state["servo"]      = data.get("servo",   ["Closed","Closed"])
                    system_state["freq"]       = data.get("freq",    [0,0,0,0])
                    system_state["sub_online"] = True
        except:
            with lock:
                system_state["sub_online"] = False
        time.sleep(0.5)

def stream_generator(feed):
    while True:
        with lock:
            frame = raw_frame if feed == "raw" else processed_frame
        if frame is None:
            placeholder = np.full((240, 320, 3), 38, dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for camera...",
                        (25, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120,120,120), 1)
            frame = placeholder
        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            time.sleep(0.05)
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        time.sleep(0.05)

# ============================================================
# Routes
# ============================================================
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video/raw")
def video_raw():
    return Response(stream_generator("raw"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/video/processed")
def video_processed():
    return Response(stream_generator("processed"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/api/status")
def api_status():
    with lock:
        return jsonify({
            "parking":    list(system_state["parking"]),
            "servo":      list(system_state["servo"]),
            "freq":       list(system_state["freq"]),
            "last_seen":  system_state["last_seen"],
            "cam_online": system_state["cam_online"],
            "sub_online": system_state["sub_online"],
            "time":       datetime.now().strftime("%H:%M:%S"),
            "date":       datetime.now().strftime("%d %B %Y"),
        })

@app.route("/api/gate/<gate_type>")
def api_gate(gate_type):
    if gate_type in ("in", "out"):
        trigger_gate(gate_type)
        return jsonify({"ok": True})
    return jsonify({"ok": False}), 400

if __name__ == "__main__":
    print("🚀 UNMC Parking Management System")
    threading.Thread(target=camera_grabber, daemon=True).start()
    threading.Thread(target=sub_esp_poller, daemon=True).start()
    print("🌐 Dashboard → http://localhost:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True)