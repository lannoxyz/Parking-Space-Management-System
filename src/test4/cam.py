"""
cam.py  –  YUV422 UDP Receiver & MJPEG Streaming Server
=========================================================
Receives chunked UDP packets from two ESP32 cameras,
reassembles full frames, converts YUV422 → BGR,
and serves live MJPEG streams via Flask on port 5000.

UDP Packet Format (agreed with ESP32 firmware):
  [0]       : cam_id        (uint8,  1 = Entrance, 2 = Exit)
  [1-2]     : frame_seq     (uint16, rolling frame counter)
  [3-4]     : chunk_index   (uint16, 0-based)
  [5-6]     : total_chunks  (uint16, chunks per frame)
  [7...]    : payload bytes

Resolution : 320×240 YUV422  →  153 600 bytes per frame
Chunk size : 1400 bytes  →  110 chunks per frame
"""

import socket
import threading
import struct
import time
import numpy as np
import cv2
from flask import Flask, Response, render_template_string

# ──────────────────────────────────────────
#  Configuration
# ──────────────────────────────────────────
WIDTH       = 320
HEIGHT      = 240
FRAME_BYTES = WIDTH * HEIGHT * 2   # 153 600

CHUNK_SIZE  = 1400                 # max safe UDP payload
TOTAL_CHUNKS = (FRAME_BYTES + CHUNK_SIZE - 1) // CHUNK_SIZE  # 110

RECV_PORT_CAM1 = 5001
RECV_PORT_CAM2 = 5002
FLASK_PORT     = 5000

HEADER_FMT  = ">BHHH"   # cam_id(1), frame_seq(2), chunk_idx(2), total(2)
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 7 bytes

# ──────────────────────────────────────────
#  Shared Frame Buffers
# ──────────────────────────────────────────
_lock = threading.Lock()
latest_bgr = {
    "cam1": np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8),
    "cam2": np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8),
}

# ──────────────────────────────────────────
#  YUV422 (YUYV) → BGR
# ──────────────────────────────────────────
def yuv422_to_bgr(raw_bytes: bytes) -> np.ndarray:
    """Convert raw YUYV bytes to a BGR numpy array."""
    yuv = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((HEIGHT, WIDTH * 2))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)

# ──────────────────────────────────────────
#  UDP Receiver Thread
# ──────────────────────────────────────────
def udp_receiver(port: int, cam_key: str) -> None:
    """
    Listen on `port`, reassemble chunked UDP packets into full YUV422
    frames, convert to BGR, and store in latest_bgr[cam_key].
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)  # 4 MB recv buffer
    sock.bind(("0.0.0.0", port))
    sock.settimeout(5.0)

    print(f"[{cam_key}] UDP receiver listening on port {port}")

    # Reassembly state  { frame_seq: bytearray of FRAME_BYTES }
    reassembly: dict[int, bytearray] = {}
    received_chunks: dict[int, set] = {}
    frame_timestamps: dict[int, float] = {}

    while True:
        try:
            data, _ = sock.recvfrom(CHUNK_SIZE + HEADER_SIZE + 16)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[{cam_key}] Socket error: {e}")
            time.sleep(0.5)
            continue

        if len(data) < HEADER_SIZE:
            continue  # malformed

        # ── Parse header ──
        cam_id, frame_seq, chunk_idx, total = struct.unpack_from(HEADER_FMT, data, 0)
        payload = data[HEADER_SIZE:]

        # ── Init reassembly buffer for new frame ──
        if frame_seq not in reassembly:
            reassembly[frame_seq]        = bytearray(FRAME_BYTES)
            received_chunks[frame_seq]   = set()
            frame_timestamps[frame_seq]  = time.time()

        # ── Write chunk into buffer ──
        offset = chunk_idx * CHUNK_SIZE
        end    = min(offset + len(payload), FRAME_BYTES)
        reassembly[frame_seq][offset:end] = payload[:end - offset]
        received_chunks[frame_seq].add(chunk_idx)

        # ── Check if all chunks arrived ──
        if len(received_chunks[frame_seq]) >= total:
            bgr = yuv422_to_bgr(bytes(reassembly[frame_seq]))
            with _lock:
                latest_bgr[cam_key] = bgr
            # Clean up this frame
            del reassembly[frame_seq]
            del received_chunks[frame_seq]
            del frame_timestamps[frame_seq]

        # ── Purge stale incomplete frames (older than 0.5 s) ──
        stale = [seq for seq, ts in frame_timestamps.items()
                 if time.time() - ts > 0.5]
        for seq in stale:
            reassembly.pop(seq, None)
            received_chunks.pop(seq, None)
            frame_timestamps.pop(seq, None)

# ──────────────────────────────────────────
#  Flask MJPEG Generator
# ──────────────────────────────────────────
def mjpeg_generator(cam_key: str):
    while True:
        with _lock:
            frame = latest_bgr[cam_key].copy()

        ret, jpeg = cv2.imencode(
            ".jpg", frame,
            [cv2.IMWRITE_JPEG_QUALITY, 80]
        )
        if not ret:
            time.sleep(0.01)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )
        time.sleep(0.033)  # ~30 FPS cap

# ──────────────────────────────────────────
#  Flask App
# ──────────────────────────────────────────
app = Flask(__name__)

DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
  <title>Raw Camera Feed — cam.py</title>
  <style>
    body { background:#111; color:#eee; font-family:monospace; text-align:center; }
    img  { width:45%; margin:10px; border:1px solid #444; }
  </style>
</head>
<body>
  <h2>ESP32 Raw Camera Streams (cam.py)</h2>
  <p>Entrance (CAM1) → CV pipeline → <code>cv.py :5050</code></p>
  <img src="/cam1" alt="CAM1 offline">
  <img src="/cam2" alt="CAM2 offline">
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
    threading.Thread(
        target=udp_receiver, args=(RECV_PORT_CAM1, "cam1"), daemon=True
    ).start()
    threading.Thread(
        target=udp_receiver, args=(RECV_PORT_CAM2, "cam2"), daemon=True
    ).start()

    print(f"[cam.py] MJPEG streams available at http://localhost:{FLASK_PORT}/cam1 and /cam2")
    app.run(host="0.0.0.0", port=FLASK_PORT, debug=False, threaded=True)