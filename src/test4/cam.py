"""
cam.py  –  YUV422 UDP Receiver & MJPEG Streaming Server
=========================================================
Receives chunked UDP packets from two ESP32 cameras,
reassembles full frames, converts YUV422 → BGR,
and serves live MJPEG streams via Flask on port 5000.

UDP Packet Format (7-byte header, big-endian):
  [0]   : cam_id        (uint8,  1 = Entrance/CAM1, 2 = Exit/CAM2)
  [1-2] : frame_seq     (uint16, rolling frame counter)
  [3-4] : chunk_index   (uint16, 0-based)
  [5-6] : total_chunks  (uint16, chunks per frame)
  [7..] : payload bytes

CAM1 resolution : 320x240  YUV422  -> 153,600 bytes/frame -> 110 chunks
CAM2 resolution : 160x120  YUV422  ->  38,400 bytes/frame ->  28 chunks
"""

import socket
import threading
import struct
import time
import numpy as np
import cv2
from flask import Flask, Response

# ──────────────────────────────────────────
#  Per-Camera Configuration
#  BUG FIX #3: original code had a single global WIDTH/HEIGHT (320x240)
#              but CAM2 is QQVGA (160x120).  Wrong buffer size caused
#              corrupt reassembly and the cv2 reshape crash.
# ──────────────────────────────────────────
CAM_CONFIG = {
    "cam1": {"width": 320, "height": 240},
    "cam2": {"width": 160, "height": 120},
}

def frame_bytes(cam_key: str) -> int:
    c = CAM_CONFIG[cam_key]
    return c["width"] * c["height"] * 2   # YUV422 = 2 bytes per pixel

CHUNK_SIZE  = 1400
HEADER_FMT  = ">BHHH"                          # cam_id(1), seq(2), chunk(2), total(2)
HEADER_SIZE = struct.calcsize(HEADER_FMT)       # 7 bytes

RECV_PORT_CAM1 = 5001
RECV_PORT_CAM2 = 5002
FLASK_PORT     = 5000

# ──────────────────────────────────────────
#  Shared Frame Buffers (BGR, one per camera)
# ──────────────────────────────────────────
_lock = threading.Lock()
latest_bgr = {
    "cam1": np.zeros(
        (CAM_CONFIG["cam1"]["height"], CAM_CONFIG["cam1"]["width"], 3), dtype=np.uint8
    ),
    "cam2": np.zeros(
        (CAM_CONFIG["cam2"]["height"], CAM_CONFIG["cam2"]["width"], 3), dtype=np.uint8
    ),
}

# ──────────────────────────────────────────
#  YUV422 (YUYV) -> BGR
#
#  BUG FIX #2: original reshape was (HEIGHT, WIDTH * 2)
#              -> shape (240, 640) -> treated as single-channel 2D array
#              -> cv2.COLOR_YUV2BGR_YUYV requires 2-channel input
#              -> crash: "Bad number of channels / scn is 1"
#
#  Fix: reshape to (HEIGHT, WIDTH, 2) -> shape (240, 320, 2) = 2-channel
# ──────────────────────────────────────────
def yuv422_to_bgr(raw_bytes: bytes, width: int, height: int) -> np.ndarray:
    yuv = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((height, width, 2))  # <- FIX
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)

# ──────────────────────────────────────────
#  UDP Receiver Thread
# ──────────────────────────────────────────
def udp_receiver(port: int, cam_key: str) -> None:
    cfg      = CAM_CONFIG[cam_key]
    width    = cfg["width"]
    height   = cfg["height"]
    total_fb = frame_bytes(cam_key)
    n_chunks = (total_fb + CHUNK_SIZE - 1) // CHUNK_SIZE

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(5.0)

    print(f"[{cam_key}] UDP receiver on port {port}  "
          f"({width}x{height}, {total_fb} B/frame, {n_chunks} chunks/frame)")

    reassembly:       dict[int, bytearray] = {}
    received_chunks:  dict[int, set]       = {}
    frame_timestamps: dict[int, float]     = {}

    frames_ok    = 0
    frames_stale = 0
    last_report  = time.time()

    while True:
        try:
            data, _ = sock.recvfrom(CHUNK_SIZE + HEADER_SIZE + 32)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[{cam_key}] Socket error: {e}")
            time.sleep(0.1)
            continue

        if len(data) < HEADER_SIZE:
            continue

        # ── Parse header ──
        cam_id, frame_seq, chunk_idx, total = struct.unpack_from(HEADER_FMT, data, 0)
        payload = data[HEADER_SIZE:]

        if chunk_idx >= total or total == 0:
            continue

        # ── Init reassembly buffer for new frame ──
        if frame_seq not in reassembly:
            reassembly[frame_seq]       = bytearray(total_fb)
            received_chunks[frame_seq]  = set()
            frame_timestamps[frame_seq] = time.time()

        # ── Write chunk into correct position ──
        offset = chunk_idx * CHUNK_SIZE
        end    = min(offset + len(payload), total_fb)
        reassembly[frame_seq][offset:end] = payload[:end - offset]
        received_chunks[frame_seq].add(chunk_idx)

        # ── Frame complete? ──
        if len(received_chunks[frame_seq]) >= total:
            try:
                bgr = yuv422_to_bgr(bytes(reassembly[frame_seq]), width, height)
                with _lock:
                    latest_bgr[cam_key] = bgr
                frames_ok += 1
            except Exception as e:
                print(f"[{cam_key}] Convert error frame {frame_seq}: {e}")
            finally:
                reassembly.pop(frame_seq, None)
                received_chunks.pop(frame_seq, None)
                frame_timestamps.pop(frame_seq, None)

        # ── Purge stale frames (> 0.5 s old) ──
        stale = [seq for seq, ts in list(frame_timestamps.items())
                 if time.time() - ts > 0.5]
        for seq in stale:
            reassembly.pop(seq, None)
            received_chunks.pop(seq, None)
            frame_timestamps.pop(seq, None)
            frames_stale += 1

        # ── Stats every 10 s ──
        now = time.time()
        if now - last_report >= 10.0:
            print(f"[{cam_key}] decoded={frames_ok}  stale_dropped={frames_stale}  "
                  f"pending={len(reassembly)}")
            frames_ok    = 0
            frames_stale = 0
            last_report  = now

# ──────────────────────────────────────────
#  Flask MJPEG Generator
# ──────────────────────────────────────────
def mjpeg_generator(cam_key: str):
    while True:
        with _lock:
            frame = latest_bgr[cam_key].copy()

        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            time.sleep(0.01)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )
        time.sleep(0.033)   # ~30 FPS cap

# ──────────────────────────────────────────
#  Flask App
# ──────────────────────────────────────────
app = Flask(__name__)

DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Camera Feeds</title>
  <style>
    body  { background:#111; color:#eee; font-family:monospace; text-align:center; margin:0; padding:20px; }
    h2    { color:#4af; }
    p     { color:#888; font-size:13px; margin:4px 0 20px; }
    .feeds { display:flex; justify-content:center; gap:24px; flex-wrap:wrap; }
    .cam  { border:1px solid #333; padding:12px; border-radius:6px; background:#1a1a1a; }
    .cam h3 { margin:0 0 8px; color:#adf; font-size:13px; }
    img   { display:block; border:1px solid #444; }
  </style>
</head>
<body>
  <h2>ESP32 Raw Camera Streams &mdash; cam.py</h2>
  <p>Entrance CAM1 (320&times;240) &nbsp;&nbsp;|&nbsp;&nbsp; Exit CAM2 (160&times;120)</p>
  <div class="feeds">
    <div class="cam">
      <h3>CAM1 &mdash; Entrance</h3>
      <img src="/cam1" alt="CAM1 offline" width="320" height="240">
    </div>
    <div class="cam">
      <h3>CAM2 &mdash; Exit</h3>
      <img src="/cam2" alt="CAM2 offline" width="160" height="120">
    </div>
  </div>
</body>
</html>"""

@app.route("/")
def index():
    return DASHBOARD_HTML

@app.route("/cam1")
def stream_cam1():
    return Response(
        mjpeg_generator("cam1"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/cam2")
def stream_cam2():
    return Response(
        mjpeg_generator("cam2"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

# ──────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=udp_receiver, args=(RECV_PORT_CAM1, "cam1"), daemon=True).start()
    threading.Thread(target=udp_receiver, args=(RECV_PORT_CAM2, "cam2"), daemon=True).start()

    print("=" * 58)
    print("  cam.py  —  ESP32 YUV422 UDP receiver + MJPEG server")
    print("=" * 58)
    print(f"  CAM1  port : {RECV_PORT_CAM1}   320x240  153600 B/frame  110 chunks")
    print(f"  CAM2  port : {RECV_PORT_CAM2}   160x120   38400 B/frame   28 chunks")
    print(f"  Dashboard  : http://localhost:{FLASK_PORT}/")
    print(f"  CAM1 feed  : http://localhost:{FLASK_PORT}/cam1")
    print(f"  CAM2 feed  : http://localhost:{FLASK_PORT}/cam2")
    print("=" * 58)

    app.run(host="0.0.0.0", port=FLASK_PORT, debug=False, threaded=True)
