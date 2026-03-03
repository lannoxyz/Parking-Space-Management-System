"""
cam.py — YUV422 UDP receiver + MJPEG web server
CAM1: 320×240  port 5001
CAM2: 160×120  port 5002
"""

import socket, threading, struct, time
import numpy as np
import cv2
from flask import Flask, Response

# ── Config ────────────────────────────────────────────────────────────────
CAM_CFG = {
    "cam1": {"w": 320, "h": 240, "port": 5001},
    "cam2": {"w": 160, "h": 120, "port": 5002},
}
CHUNK   = 1400
HDR_FMT = ">BHHH"
HDR_SZ  = struct.calcsize(HDR_FMT)   # 7
FLASK_PORT = 5000

# ── Shared frames ─────────────────────────────────────────────────────────
lock = threading.Lock()
latest = {k: np.zeros((v["h"], v["w"], 3), dtype=np.uint8) for k, v in CAM_CFG.items()}

# ── YUV decode (try YUYV first, fall back to UYVY if image looks wrong) ──
def decode(raw: bytes, w: int, h: int) -> np.ndarray:
    arr = np.frombuffer(raw, np.uint8).reshape((h, w, 2))
    # YUYV is OV7670 default
    bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
    return bgr

# ── UDP receiver ──────────────────────────────────────────────────────────
def receiver(key: str):
    cfg   = CAM_CFG[key]
    w, h  = cfg["w"], cfg["h"]
    port  = cfg["port"]
    fbytes = w * h * 2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(5.0)
    print(f"[{key}] listening port {port}  ({w}×{h}  {fbytes}B/frame)")

    asm:   dict[int, bytearray] = {}
    rcvd:  dict[int, set]       = {}
    ts:    dict[int, float]     = {}

    ok_cnt = stale_cnt = 0
    t_report = time.time()

    while True:
        try:
            data, _ = sock.recvfrom(CHUNK + HDR_SZ + 32)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[{key}] socket error: {e}")
            time.sleep(0.1)
            continue

        if len(data) < HDR_SZ:
            continue

        _, seq, idx, total = struct.unpack_from(HDR_FMT, data)
        payload = data[HDR_SZ:]
        if total == 0 or idx >= total:
            continue

        if seq not in asm:
            asm[seq]  = bytearray(fbytes)
            rcvd[seq] = set()
            ts[seq]   = time.time()

        off = idx * CHUNK
        end = min(off + len(payload), fbytes)
        asm[seq][off:end] = payload[:end-off]
        rcvd[seq].add(idx)

        if len(rcvd[seq]) >= total:
            try:
                bgr = decode(bytes(asm[seq]), w, h)
                with lock:
                    latest[key] = bgr
                ok_cnt += 1
            except Exception as e:
                print(f"[{key}] decode error seq={seq}: {e}")
            asm.pop(seq, None); rcvd.pop(seq, None); ts.pop(seq, None)

        # purge stale (>0.5s)
        stale = [s for s, t_ in list(ts.items()) if time.time()-t_ > 0.5]
        for s in stale:
            asm.pop(s,None); rcvd.pop(s,None); ts.pop(s,None)
            stale_cnt += 1

        now = time.time()
        if now - t_report >= 10:
            print(f"[{key}] decoded={ok_cnt} stale={stale_cnt} pending={len(asm)}")
            ok_cnt = stale_cnt = 0
            t_report = now

# ── MJPEG ────────────────────────────────────────────────────────────────
def stream(key: str):
    while True:
        with lock:
            frm = latest[key].copy()
        ok, jpg = cv2.imencode(".jpg", frm, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
        time.sleep(0.033)

# ── Flask ────────────────────────────────────────────────────────────────
app = Flask(__name__)

HTML = """<!DOCTYPE html><html><head><title>Cameras</title>
<style>
body{background:#111;color:#eee;font-family:monospace;text-align:center;padding:20px}
h2{color:#4af}.wrap{display:flex;justify-content:center;gap:20px;flex-wrap:wrap}
.box{border:1px solid #333;padding:12px;border-radius:8px;background:#1a1a1a}
.box h3{margin:0 0 8px;color:#adf;font-size:13px}img{display:block;border:1px solid #444}
</style></head><body>
<h2>ESP32 Camera Streams</h2>
<div class="wrap">
  <div class="box"><h3>CAM1 — Entrance (320×240)</h3>
    <img src="/cam1" width="320" height="240" onerror="this.alt='offline'"></div>
  <div class="box"><h3>CAM2 — Exit (160×120)</h3>
    <img src="/cam2" width="160" height="120" onerror="this.alt='offline'"></div>
</div></body></html>"""

@app.route("/")
def index(): return HTML

@app.route("/cam1")
def s1(): return Response(stream("cam1"), mimetype="multipart/x-mixed-replace; boundary=f")

@app.route("/cam2")
def s2(): return Response(stream("cam2"), mimetype="multipart/x-mixed-replace; boundary=f")

# ── Main ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    for k in CAM_CFG:
        threading.Thread(target=receiver, args=(k,), daemon=True).start()

    print("="*50)
    print("  cam.py — UDP receiver + MJPEG server")
    print(f"  CAM1: port 5001  320×240  110 chunks/frame")
    print(f"  CAM2: port 5002  160×120   28 chunks/frame")
    print(f"  Web:  http://localhost:{FLASK_PORT}/")
    print("="*50)
    app.run(host="0.0.0.0", port=FLASK_PORT, debug=False, threaded=True)
