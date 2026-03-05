import cv2
import threading
import time
import requests
import numpy as np
from flask import Flask, Response, jsonify, render_template
from datetime import datetime
from ultralytics import YOLO

app = Flask(__name__, template_folder='templates', static_folder='static')

# ESP IP addresses
MAIN_ESP_URL = "http://172.20.10.3"   # Camera ESP (ESP32-S3)
SUB_ESP_URL  = "http://172.20.10.4"   # Control ESP (servo + parking sensors)

print("⌛ Loading YOLOv8 model...")
model = YOLO('yolov8n.pt')
# COCO classes: 2=car, 3=motorcycle, 5=bus, 7=truck
VEHICLE_CLASSES = [2, 3, 5, 7]

lock = threading.Lock()
raw_frame       = None
processed_frame = None

system_state = {
    "parking":    [False] * 4,
    "servo":      "Closed",     # scalar string — single gate
    "cam_online": False,
    "sub_online": False,
    "last_seen":  "Never",
    "freq":       [0, 0, 0, 0],
}


# ---------------------------------------------------------------------------
# CV / YOLO pipeline
# ---------------------------------------------------------------------------

def run_cv_pipeline(frame: np.ndarray):
    """
    Run YOLOv8 on the frame and draw bounding boxes.
    Returns (annotated_bgr_frame, vehicle_detected: bool).
    """
    if frame is None:
        return None, False

    if len(frame.shape) == 2:
        bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif frame.shape[2] == 4:
        bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    else:
        bgr = frame.copy()

    h, w = bgr.shape[:2]
    if w < 320 or h < 240:
        scale = max(320 / w, 240 / h)
        bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)))

    results = model.predict(
        source=bgr,
        conf=0.25,
        classes=VEHICLE_CLASSES,
        verbose=False,
        imgsz=320,
    )

    vehicle_detected = False
    for r in results:
        boxes = r.boxes
        if boxes is None:
            continue
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = f"{model.names[cls]} {conf:.2f}"

            cv2.rectangle(bgr, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv2.putText(
                bgr, label,
                (x1, max(y1 - 8, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
            )
            vehicle_detected = True

    return bgr, vehicle_detected


# ---------------------------------------------------------------------------
# Gate state machine
#
#  IDLE ──(vehicle detected 1 s)──► CONFIRMING ──(confirmed)──► OPEN
#                                                                  │
#                                       IDLE ◄──(5 s elapsed)─────┘
#
#  While in OPEN state:
#    - YOLO pipeline still runs so the processed feed stays live,
#      but detections are ignored (gate logic is frozen).
#    - No requests are sent to sub-ESP (it manages its own close timer).
# ---------------------------------------------------------------------------

GATE_CONFIRM_S = 1.0   # vehicle must be present this long before opening
GATE_OPEN_S    = 5.0   # how long to stay in OPEN/cooldown before returning to IDLE

# Gate FSM state  — accessed only from camera_grabber thread, no lock needed
_gate_state         = "IDLE"      # "IDLE" | "CONFIRMING" | "OPEN"
_gate_detect_since  = None        # time.time() when vehicle first appeared
_gate_opened_at     = None        # time.time() when gate was triggered


def _gate_fsm_tick(vehicle_detected: bool):
    """
    Call once per camera frame with whether a vehicle is currently visible.
    Manages state transitions and fires trigger_gate() exactly once per cycle.
    """
    global _gate_state, _gate_detect_since, _gate_opened_at

    now = time.time()

    if _gate_state == "IDLE":
        if vehicle_detected:
            _gate_state        = "CONFIRMING"
            _gate_detect_since = now
            print("[GATE] Vehicle detected — confirming...")
        # no vehicle → stay IDLE

    elif _gate_state == "CONFIRMING":
        if vehicle_detected:
            if now - _gate_detect_since >= GATE_CONFIRM_S:
                # Confirmed — open gate
                _gate_state    = "OPEN"
                _gate_opened_at = now
                trigger_gate()
                print("[GATE] Confirmed → OPEN")
        else:
            # Vehicle disappeared before confirmation — reset
            _gate_state        = "IDLE"
            _gate_detect_since = None
            print("[GATE] Lost before confirm — back to IDLE")

    elif _gate_state == "OPEN":
        # Frozen for GATE_OPEN_S seconds regardless of detections
        if now - _gate_opened_at >= GATE_OPEN_S:
            _gate_state        = "IDLE"
            _gate_detect_since = None
            _gate_opened_at    = None
            print("[GATE] Cooldown done — back to IDLE")


def trigger_gate():
    """Send a single open command to the sub-ESP (non-blocking)."""
    def _send():
        try:
            requests.get(f"{SUB_ESP_URL}/action?servo=in", timeout=2)
            print("[GATE] Trigger sent to sub-ESP")
        except Exception as e:
            print(f"[GATE] Trigger failed: {e}")
    threading.Thread(target=_send, daemon=True).start()


# ---------------------------------------------------------------------------
# Status poll from sub-ESP
# ---------------------------------------------------------------------------

def sub_status_poller():
    """Continuously poll the sub-ESP for parking + servo state."""
    while True:
        try:
            r = requests.get(f"{SUB_ESP_URL}/status", timeout=2)
            if r.status_code == 200:
                d = r.json()
                with lock:
                    system_state["parking"]    = d.get("parking", [False] * 4)
                    # sub-ESP now returns servo as a scalar string
                    servo_raw = d.get("servo", "Closed")
                    system_state["servo"]      = servo_raw if isinstance(servo_raw, str) else (servo_raw[0] if servo_raw else "Closed")
                    system_state["freq"]       = d.get("freq",    [0, 0, 0, 0])
                    system_state["sub_online"] = True
        except Exception:
            with lock:
                system_state["sub_online"] = False
        time.sleep(0.6)


# ---------------------------------------------------------------------------
# Camera grabber
# ---------------------------------------------------------------------------

def camera_grabber():
    global raw_frame, processed_frame
    url = f"{MAIN_ESP_URL.rstrip('/')}/capture"
    while True:
        try:
            r = requests.get(url, timeout=3.0)
            if r.status_code == 200:
                nparr = np.frombuffer(r.content, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    proc, vehicle_detected = run_cv_pipeline(frame)
                    _gate_fsm_tick(vehicle_detected)
                    with lock:
                        raw_frame       = frame
                        processed_frame = proc if proc is not None else frame
                        system_state["cam_online"] = True
                        system_state["last_seen"]  = datetime.now().strftime("%H:%M:%S")
        except Exception:
            with lock:
                system_state["cam_online"] = False
            time.sleep(1)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# MJPEG stream generator
# ---------------------------------------------------------------------------

def stream_generator(feed: str):
    while True:
        with lock:
            f = raw_frame if feed == "raw" else processed_frame
        if f is not None:
            ok, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + jpeg.tobytes()
                    + b'\r\n'
                )
        time.sleep(0.08)


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video/raw")
def video_raw():
    return Response(
        stream_generator("raw"),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )

@app.route("/video/processed")
def video_processed():
    return Response(
        stream_generator("processed"),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )

@app.route("/api/status")
def api_status():
    with lock:
        state = dict(system_state)
    return jsonify(state)

@app.route("/api/gate/<gate_type>")
def api_gate(gate_type):
    trigger_gate()   # single gate regardless of type param
    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    threading.Thread(target=camera_grabber,    daemon=True).start()
    threading.Thread(target=sub_status_poller, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)