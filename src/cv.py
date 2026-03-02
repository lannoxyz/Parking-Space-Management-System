import cv2
import threading
import time
import requests
import numpy as np
from flask import Flask, Response, jsonify

# ======================================
# Flask App Init (FIX: was missing entirely)
# ======================================
app = Flask(__name__)

# ======================================
# ESP32 Camera Endpoints
# ======================================
cam_1_url = "http://10.143.39.206"   # Entrance (AI + Servo)
cam_2_url = "http://10.143.39.6"     # Exit (monitor only)

# ======================================
# Shared Memory (Frame Buffers & System State)
# FIX: protected by a threading.Lock to prevent race conditions
# ======================================
frame_lock = threading.Lock()

latest_cam_1_frame = None
latest_cam_2_frame = None

system_status = {
    "parking": [False, False, False, False],
    "servo": ["Closed", "Closed"]
}

# Debounce timestamp — prevents repeated open commands
last_gate_open_time = 0

# ======================================
# Simulated YOLO Logic & Action Trigger
# ======================================
def run_yolo_and_trigger(frame):
    global last_gate_open_time

    # ---- Placeholder: replace with model.predict(frame) ----
    is_car_detected = False

    # Simulate random vehicle detection for testing
    if np.random.rand() > 0.98:
        is_car_detected = True

    current_time = time.time()

    # If car detected and cooldown has passed (6s > ESP32's 5s hardware timeout)
    if is_car_detected and (current_time - last_gate_open_time > 6.0):
        print("🚗 AI Detected Vehicle! Sending command to ESP32...")

        # Async HTTP request to ESP32 so it doesn't block the grabber thread
        def trigger_hardware():
            try:
                requests.get(f"{cam_1_url}/action?type=enter", timeout=2)
            except Exception as e:
                print(f"⚠️ Failed to open entrance gate: {e}")

        threading.Thread(target=trigger_hardware, daemon=True).start()

        # Update shared status for the frontend
        with frame_lock:
            system_status["servo"][0] = "Open"
        last_gate_open_time = current_time

    # FIX: reset threshold is 6.5s (slightly longer than ESP32's 5s physical close)
    # This ensures the physical gate always finishes closing before the UI reflects it
    with frame_lock:
        servo_state = system_status["servo"][0]

    if servo_state == "Open" and (current_time - last_gate_open_time > 6.5):
        with frame_lock:
            system_status["servo"][0] = "Closed"

    return frame

# ======================================
# Grabber Thread
# ======================================
def grab_thread(base_url, cam_id):
    global latest_cam_1_frame, latest_cam_2_frame

    consecutive_failures = 0

    while True:
        try:
            capture_url = f"{base_url}/capture"
            r = requests.get(capture_url, timeout=1)

            if r.status_code == 200:
                jpg_data = np.frombuffer(r.content, dtype=np.uint8)
                frame = cv2.imdecode(jpg_data, cv2.IMREAD_COLOR)

                if frame is None:
                    raise ValueError("cv2.imdecode returned None — bad JPEG data")

                # Only entrance camera (CAM 1) runs AI detection
                if cam_id == 1:
                    frame = run_yolo_and_trigger(frame)
                    with frame_lock:
                        latest_cam_1_frame = frame
                else:
                    with frame_lock:
                        latest_cam_2_frame = frame

                consecutive_failures = 0  # reset on success

        # FIX: log errors instead of silently swallowing them
        except Exception as e:
            consecutive_failures += 1
            print(f"[CAM {cam_id}] Error (failure #{consecutive_failures}): {e}")
            if consecutive_failures >= 10:
                print(f"[CAM {cam_id}] ⚠️ Camera appears offline after {consecutive_failures} consecutive failures.")

        time.sleep(0.05)  # ~20 FPS cap to reduce ESP32 load

# ======================================
# MJPEG Stream Generator
# ======================================
def generate_stream(cam_id):
    while True:
        with frame_lock:
            frame = latest_cam_1_frame if cam_id == 1 else latest_cam_2_frame

        if frame is None:
            time.sleep(0.05)
            continue

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buffer.tobytes() +
            b'\r\n'
        )
        time.sleep(0.05)

# ======================================
# Flask Routes
# ======================================
@app.route("/video")
def video_feed_1():
    return Response(generate_stream(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/video2")
def video_feed_2():
    return Response(generate_stream(2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    with frame_lock:
        # Return a copy to avoid holding the lock during JSON serialization
        current_status = {
            "parking": list(system_status["parking"]),
            "servo": list(system_status["servo"])
        }
    return jsonify(current_status)

# ======================================
# Entry Point
# ======================================
if __name__ == "__main__":
    print("🚀 Starting Smart Parking Edge Server...")

    # Start camera grabber threads
    t1 = threading.Thread(target=grab_thread, args=(cam_1_url, 1), daemon=True)
    t2 = threading.Thread(target=grab_thread, args=(cam_2_url, 2), daemon=True)
    t1.start()
    t2.start()

    print("📷 Camera grabbers started.")
    print("🌐 Dashboard available at http://localhost:5000")

    # Run Flask (threaded=True allows concurrent requests)
    app.run(host="0.0.0.0", port=5000, threaded=True)
