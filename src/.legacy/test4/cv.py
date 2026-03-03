"""
cv.py  –  Vision Pipeline & Smart Parking Dashboard Server
===========================================================
• Grabs MJPEG streams from cam.py (localhost:5000)
• Runs YOLOv8 vehicle detection on the entrance frame
• Tracks 4 parking-slot occupancy via colour/contour analysis
• Sends servo commands to ESP32 over UDP
• Serves a polished dashboard UI on port 5050

Routes
------
  GET /            → Dashboard HTML
  GET /video1      → Processed entrance MJPEG stream
  GET /video2      → Processed exit MJPEG stream
  GET /status      → JSON { parking:[bool×4], servo:[str×2] }
"""

import cv2
import threading
import time
import socket
import struct
import numpy as np
from flask import Flask, Response, jsonify, render_template_string

# ──────────────────────────────────────────
#  Optional YOLOv8 Import (graceful fallback)
# ──────────────────────────────────────────
try:
    from ultralytics import YOLO
    yolo_model = YOLO("yolov8n.pt")   # nano – fastest; swap for yolov8s.pt for better accuracy
    YOLO_AVAILABLE = True
    print("[cv.py] YOLOv8 loaded successfully.")
except Exception as e:
    YOLO_AVAILABLE = False
    yolo_model = None
    print(f"[cv.py] YOLOv8 unavailable ({e}). Running in simulation mode.")

# ──────────────────────────────────────────
#  Configuration
# ──────────────────────────────────────────
CAM_PY_URL_1  = "http://localhost:5000/cam1"   # entrance MJPEG
CAM_PY_URL_2  = "http://localhost:5000/cam2"   # exit MJPEG

ESP32_CAM1_IP   = "10.143.39.206"              # Entrance ESP32
ESP32_CMD_PORT  = 6001                          # UDP command port on ESP32

FLASK_PORT    = 5050

# YOLO class IDs considered as "vehicle"
VEHICLE_CLASSES = {2, 3, 5, 7}   # car, motorcycle, bus, truck

# Parking slot ROIs: (x1, y1, x2, y2) in 320×240 frame
# ─── Adjust these to match your physical parking layout ───
PARKING_ROIS = [
    (10,  60, 90,  180),   # Slot 1
    (100, 60, 180, 180),   # Slot 2
    (190, 60, 270, 180),   # Slot 3
    (280, 60, 319, 180),   # Slot 4
]

# Gate auto-close delay (seconds) – must exceed ESP32 hardware timeout
GATE_OPEN_DURATION = 6.5

# ──────────────────────────────────────────
#  Shared State
# ──────────────────────────────────────────
_lock = threading.Lock()

latest_frames = {
    "cam1": None,   # processed entrance frame
    "cam2": None,   # processed exit frame
}

system_status = {
    "parking": [False, False, False, False],
    "servo":   ["Closed", "Closed"],
}

last_gate_open_time = 0.0   # debounce for entrance gate

# ──────────────────────────────────────────
#  UDP Command Sender (PC → ESP32)
# ──────────────────────────────────────────
_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_esp32_command(cmd: str, ip: str = ESP32_CAM1_IP, port: int = ESP32_CMD_PORT) -> None:
    """Send a text command string to the ESP32 via UDP (fire-and-forget)."""
    try:
        _udp_sock.sendto(cmd.encode(), (ip, port))
        print(f"[cv.py] → ESP32 command: {cmd}")
    except Exception as e:
        print(f"[cv.py] ⚠ UDP send failed: {e}")

def open_entrance_gate() -> None:
    """Send servo open command (90°) to entrance gate."""
    send_esp32_command("SE1:90")
    threading.Timer(GATE_OPEN_DURATION, close_entrance_gate).start()

def close_entrance_gate() -> None:
    """Send servo close command (0°) to entrance gate."""
    send_esp32_command("SE1:0")
    with _lock:
        system_status["servo"][0] = "Closed"

# ──────────────────────────────────────────
#  Parking-Slot Occupancy Detection
# ──────────────────────────────────────────
def detect_parking_occupancy(frame: np.ndarray) -> list[bool]:
    """
    Analyse each parking ROI using a background-subtraction heuristic:
    converts the ROI to grayscale, computes mean pixel intensity relative
    to the global background.  Replace with a dedicated model if needed.
    """
    occupied = []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    for (x1, y1, x2, y2) in PARKING_ROIS:
        roi = thresh[y1:y2, x1:x2]
        fill_ratio = cv2.countNonZero(roi) / max(roi.size, 1)
        occupied.append(fill_ratio > 0.30)   # >30 % dark pixels → car present

    return occupied

# ──────────────────────────────────────────
#  YOLO Vehicle Detection + Gate Trigger
# ──────────────────────────────────────────
def run_yolo(frame: np.ndarray) -> tuple[bool, np.ndarray]:
    """
    Run YOLOv8 on `frame`.  Returns (vehicle_detected, annotated_frame).
    Falls back to random simulation when YOLO is not installed.
    """
    if YOLO_AVAILABLE:
        results = yolo_model(frame, verbose=False)[0]
        detected = False
        annotated = frame.copy()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf   = float(box.conf[0])

            if cls_id in VEHICLE_CLASSES and conf > 0.45:
                detected = True
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = f"{results.names[cls_id]} {conf:.0%}"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 80), 2)
                cv2.putText(annotated, label, (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 80), 1)

        return detected, annotated
    else:
        # Simulation: ~2 % chance of detection per frame
        detected  = np.random.rand() > 0.98
        annotated = frame.copy()
        if detected:
            cv2.putText(annotated, "[SIM] Vehicle Detected", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 80), 1)
        return detected, annotated

# ──────────────────────────────────────────
#  Overlay Drawing
# ──────────────────────────────────────────
def draw_parking_overlay(frame: np.ndarray, occupancy: list[bool]) -> np.ndarray:
    for i, (x1, y1, x2, y2) in enumerate(PARKING_ROIS):
        color  = (0, 80, 255) if occupancy[i] else (0, 220, 100)
        label  = f"S{i+1} {'OCC' if occupancy[i] else 'FREE'}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1 + 2, y1 + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)
    return frame

def draw_hud(frame: np.ndarray, cam_label: str, gate_state: str) -> np.ndarray:
    ts = time.strftime("%H:%M:%S")
    cv2.putText(frame, f"{cam_label}  {ts}", (6, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    gate_color = (0, 220, 100) if gate_state == "Open" else (0, 80, 255)
    cv2.putText(frame, f"Gate: {gate_state}", (6, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, gate_color, 1)
    return frame

# ──────────────────────────────────────────
#  MJPEG Grabber Threads (pull from cam.py)
# ──────────────────────────────────────────
def mjpeg_grabber(url: str, cam_key: str) -> None:
    """
    Continuously read MJPEG frames from cam.py, process them,
    and update latest_frames[cam_key].
    """
    global last_gate_open_time
    consecutive_failures = 0

    print(f"[cv.py] Starting grabber for {cam_key} → {url}")

    while True:
        try:
            cap = cv2.VideoCapture(url)
            if not cap.isOpened():
                raise ConnectionError(f"Cannot open stream {url}")

            consecutive_failures = 0

            while True:
                ret, frame = cap.read()
                if not ret or frame is None:
                    raise ConnectionError("Stream read failed")

                # ── Entrance camera: YOLO + gate logic ──
                if cam_key == "cam1":
                    vehicle_detected, annotated = run_yolo(frame)
                    now = time.time()

                    if vehicle_detected and (now - last_gate_open_time > 6.0):
                        last_gate_open_time = now
                        threading.Thread(target=open_entrance_gate, daemon=True).start()
                        with _lock:
                            system_status["servo"][0] = "Open"

                    # Parking occupancy from entrance cam
                    occupancy = detect_parking_occupancy(frame)
                    with _lock:
                        system_status["parking"] = occupancy
                        gate_state = system_status["servo"][0]

                    annotated = draw_parking_overlay(annotated, occupancy)
                    annotated = draw_hud(annotated, "ENTRANCE (AI)", gate_state)

                    with _lock:
                        latest_frames["cam1"] = annotated

                # ── Exit camera: basic HUD only ──
                else:
                    with _lock:
                        gate_state = system_status["servo"][1]
                    frame = draw_hud(frame, "EXIT", gate_state)
                    with _lock:
                        latest_frames["cam2"] = frame

        except Exception as e:
            consecutive_failures += 1
            print(f"[{cam_key}] Grabber error #{consecutive_failures}: {e}")
            if consecutive_failures >= 5:
                print(f"[{cam_key}] ⚠ cam.py stream appears offline. Retrying in 3 s …")
            time.sleep(3)

# ──────────────────────────────────────────
#  Flask MJPEG Generator
# ──────────────────────────────────────────
def mjpeg_generator(cam_key: str):
    while True:
        with _lock:
            frame = latest_frames[cam_key]

        if frame is None:
            time.sleep(0.05)
            continue

        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if not ret:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )
        time.sleep(0.033)

# ──────────────────────────────────────────
#  Dashboard HTML (inline – no external files needed)
# ──────────────────────────────────────────
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>UNM Smart Parking — CV Dashboard</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow:wght@300;500;700&display=swap" rel="stylesheet">
<style>
/* ── Variables ── */
:root{
  --bg:       #060d1a;
  --surface:  #0d1b2e;
  --border:   #1a3050;
  --accent:   #00e5ff;
  --green:    #00e676;
  --red:      #ff1744;
  --amber:    #ffc400;
  --text:     #cdd9e5;
  --muted:    #4a6080;
  --mono:     'Share Tech Mono', monospace;
  --sans:     'Barlow', sans-serif;
}

*{box-sizing:border-box;margin:0;padding:0;}
body{background:var(--bg);color:var(--text);font-family:var(--sans);font-weight:300;min-height:100vh;padding:0;}

/* ── Scanline overlay ── */
body::before{
  content:'';position:fixed;inset:0;pointer-events:none;z-index:9999;
  background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,229,255,.018) 2px,rgba(0,229,255,.018) 4px);
}

/* ── Header ── */
header{
  display:flex;align-items:center;justify-content:space-between;
  padding:16px 32px;border-bottom:1px solid var(--border);
  background:linear-gradient(90deg,#060d1a 0%,#091525 100%);
}
.logo{display:flex;align-items:center;gap:12px;}
.logo-icon{width:36px;height:36px;border:2px solid var(--accent);border-radius:50%;display:grid;place-items:center;color:var(--accent);font-size:18px;}
h1{font-size:1.15rem;font-weight:700;letter-spacing:3px;text-transform:uppercase;color:#fff;}
.subtitle{font-family:var(--mono);font-size:.7rem;color:var(--muted);letter-spacing:2px;}
#clock{font-family:var(--mono);font-size:.85rem;color:var(--accent);letter-spacing:2px;}
.status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:pulse 2s infinite;}
@keyframes pulse{0%,100%{opacity:1;}50%{opacity:.4;}}

/* ── Main grid ── */
main{
  display:grid;
  grid-template-columns:1fr 380px;
  grid-template-rows:auto auto;
  gap:20px;padding:24px 32px;max-width:1400px;margin:0 auto;
}

/* ── Panels ── */
.panel{
  background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:20px;
  position:relative;overflow:hidden;
}
.panel::before{
  content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:linear-gradient(90deg,transparent,var(--accent),transparent);opacity:.5;
}
.panel-title{
  font-family:var(--mono);font-size:.7rem;letter-spacing:3px;text-transform:uppercase;
  color:var(--accent);margin-bottom:16px;display:flex;align-items:center;gap:8px;
}
.panel-title::after{content:'';flex:1;height:1px;background:var(--border);}

/* ── Camera grid ── */
.camera-panel{grid-column:1;}
.cam-grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;}
.cam-box{background:#000;border:1px solid var(--border);border-radius:6px;overflow:hidden;}
.cam-label{
  font-family:var(--mono);font-size:.65rem;letter-spacing:2px;padding:6px 10px;
  background:rgba(0,229,255,.08);color:var(--accent);border-bottom:1px solid var(--border);
  display:flex;justify-content:space-between;align-items:center;
}
.cam-label .live-badge{
  background:var(--red);color:#fff;font-size:.55rem;padding:2px 6px;border-radius:3px;
  letter-spacing:1px;animation:pulse 1.5s infinite;
}
.cam-box img{width:100%;height:auto;display:block;}

/* ── Right column ── */
.right-col{grid-column:2;display:flex;flex-direction:column;gap:20px;}

/* ── Parking slots ── */
.slot-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;}
.slot{
  border:1px solid var(--border);border-radius:6px;padding:14px 10px;
  text-align:center;transition:all .4s ease;position:relative;overflow:hidden;
}
.slot.vacant{border-color:var(--green);background:rgba(0,230,118,.06);}
.slot.occupied{border-color:var(--red);background:rgba(255,23,68,.06);}
.slot-number{font-family:var(--mono);font-size:.65rem;color:var(--muted);letter-spacing:2px;margin-bottom:8px;}
.slot-icon{font-size:1.6rem;margin-bottom:6px;line-height:1;}
.slot-badge{
  font-family:var(--mono);font-size:.7rem;letter-spacing:1px;padding:3px 10px;border-radius:20px;font-weight:500;
}
.slot.vacant .slot-badge{color:var(--green);border:1px solid var(--green);}
.slot.occupied .slot-badge{color:var(--red);border:1px solid var(--red);}

/* ── Gate status ── */
.gate-list{display:flex;flex-direction:column;gap:12px;}
.gate-row{
  display:flex;justify-content:space-between;align-items:center;
  padding:14px 16px;border:1px solid var(--border);border-radius:6px;
  background:rgba(255,255,255,.02);
}
.gate-name{font-family:var(--mono);font-size:.75rem;letter-spacing:1px;color:var(--text);}
.gate-pill{
  font-family:var(--mono);font-size:.7rem;letter-spacing:2px;
  padding:5px 14px;border-radius:4px;text-transform:uppercase;font-weight:500;
  transition:all .4s ease;
}
.gate-pill.open{background:rgba(0,230,118,.15);color:var(--green);border:1px solid var(--green);box-shadow:0 0 12px rgba(0,230,118,.2);}
.gate-pill.closed{background:rgba(255,23,68,.1);color:var(--red);border:1px solid var(--red);}

/* ── Stats bar ── */
.stats-bar{
  display:flex;gap:0;border:1px solid var(--border);border-radius:6px;overflow:hidden;
}
.stat{flex:1;padding:14px;text-align:center;border-right:1px solid var(--border);}
.stat:last-child{border-right:none;}
.stat-value{font-family:var(--mono);font-size:1.4rem;color:var(--accent);font-weight:500;}
.stat-label{font-size:.65rem;color:var(--muted);letter-spacing:2px;text-transform:uppercase;margin-top:4px;}

/* ── Responsive ── */
@media(max-width:900px){
  main{grid-template-columns:1fr;padding:16px;}
  .camera-panel,.right-col{grid-column:1;}
}
</style>
</head>
<body>

<header>
  <div class="logo">
    <div class="logo-icon">🚦</div>
    <div>
      <h1>UNM Smart Parking</h1>
      <div class="subtitle">EDGE-AI MONITORING SYSTEM · cv.py</div>
    </div>
  </div>
  <div style="display:flex;align-items:center;gap:16px;">
    <span class="status-dot"></span>
    <span id="clock" style="font-size:.8rem;">--:--:--</span>
  </div>
</header>

<main>

  <!-- Camera Feeds -->
  <section class="panel camera-panel">
    <div class="panel-title">📷 Live Vision Feed</div>
    <div class="cam-grid">
      <div class="cam-box">
        <div class="cam-label">ENTRANCE · AI DETECTION <span class="live-badge">LIVE</span></div>
        <img src="/video1" alt="Entrance Offline" onerror="this.style.filter='grayscale(1)'">
      </div>
      <div class="cam-box">
        <div class="cam-label">EXIT · MONITOR <span class="live-badge">LIVE</span></div>
        <img src="/video2" alt="Exit Offline" onerror="this.style.filter='grayscale(1)'">
      </div>
    </div>
  </section>

  <!-- Right Column -->
  <div class="right-col">

    <!-- Stats -->
    <section class="panel" style="padding:0;">
      <div class="stats-bar">
        <div class="stat">
          <div class="stat-value" id="stat-free">4</div>
          <div class="stat-label">Free</div>
        </div>
        <div class="stat">
          <div class="stat-value" id="stat-occ" style="color:var(--red);">0</div>
          <div class="stat-label">Occupied</div>
        </div>
        <div class="stat">
          <div class="stat-value">4</div>
          <div class="stat-label">Total</div>
        </div>
      </div>
    </section>

    <!-- Parking Slots -->
    <section class="panel">
      <div class="panel-title">🅿 Parking Slots</div>
      <div class="slot-grid">
        <div class="slot vacant" id="slot_0"><div class="slot-number">SLOT 01</div><div class="slot-icon">🟢</div><div class="slot-badge">VACANT</div></div>
        <div class="slot vacant" id="slot_1"><div class="slot-number">SLOT 02</div><div class="slot-icon">🟢</div><div class="slot-badge">VACANT</div></div>
        <div class="slot vacant" id="slot_2"><div class="slot-number">SLOT 03</div><div class="slot-icon">🟢</div><div class="slot-badge">VACANT</div></div>
        <div class="slot vacant" id="slot_3"><div class="slot-number">SLOT 04</div><div class="slot-icon">🟢</div><div class="slot-badge">VACANT</div></div>
      </div>
    </section>

    <!-- Gate Status -->
    <section class="panel">
      <div class="panel-title">🚧 Gate Control</div>
      <div class="gate-list">
        <div class="gate-row">
          <span class="gate-name">⬆ ENTRANCE GATE</span>
          <div class="gate-pill closed" id="gate_entrance">CLOSED</div>
        </div>
        <div class="gate-row">
          <span class="gate-name">⬇ EXIT GATE</span>
          <div class="gate-pill closed" id="gate_exit">CLOSED</div>
        </div>
      </div>
    </section>

  </div>
</main>

<script>
// ── Clock ──
function tick(){document.getElementById('clock').textContent=new Date().toLocaleTimeString('en-GB');}
tick();setInterval(tick,1000);

// ── Polling ──
async function poll(){
  try{
    const r=await fetch('/status');
    if(!r.ok)return;
    const d=await r.json();

    // Slots
    let free=0,occ=0;
    for(let i=0;i<4;i++){
      const el=document.getElementById('slot_'+i);
      if(!el)continue;
      const isOcc=d.parking[i];
      isOcc?occ++:free++;
      el.className='slot '+(isOcc?'occupied':'vacant');
      el.querySelector('.slot-icon').textContent=isOcc?'🔴':'🟢';
      el.querySelector('.slot-badge').textContent=isOcc?'OCCUPIED':'VACANT';
    }
    document.getElementById('stat-free').textContent=free;
    document.getElementById('stat-occ').textContent=occ;

    // Gates
    const gates=[['gate_entrance',d.servo[0]],['gate_exit',d.servo[1]]];
    for(const[id,state]of gates){
      const el=document.getElementById(id);
      if(!el)continue;
      const isOpen=state==='Open';
      el.textContent=state.toUpperCase();
      el.className='gate-pill '+(isOpen?'open':'closed');
    }
  }catch(e){console.warn('Poll error:',e);}
}
setInterval(poll,500);
poll();
</script>
</body>
</html>"""

# ──────────────────────────────────────────
#  Flask App
# ──────────────────────────────────────────
app = Flask(__name__)

@app.route("/")
def dashboard():
    return DASHBOARD_HTML

@app.route("/video1")
def video1():
    return Response(
        mjpeg_generator("cam1"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/video2")
def video2():
    return Response(
        mjpeg_generator("cam2"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/status")
def status():
    with _lock:
        payload = {
            "parking": list(system_status["parking"]),
            "servo":   list(system_status["servo"]),
        }
    return jsonify(payload)

# ──────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────
if __name__ == "__main__":
    print("🚀 [cv.py] Starting CV pipeline…")

    threading.Thread(target=mjpeg_grabber, args=(CAM_PY_URL_1, "cam1"), daemon=True).start()
    threading.Thread(target=mjpeg_grabber, args=(CAM_PY_URL_2, "cam2"), daemon=True).start()

    print(f"📺 Dashboard → http://localhost:{FLASK_PORT}")
    app.run(host="0.0.0.0", port=FLASK_PORT, debug=False, threaded=True)