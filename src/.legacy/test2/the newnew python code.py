import cv2
import numpy as np
import requests
from ultralytics import YOLO
import time
import threading
from flask import Flask, Response, jsonify, send_from_directory
import os

# ================= 配置文件 (Configuration) =================
esp32_ip = "192.168.0.25"
url_capture = f"http://{esp32_ip}/capture"
url_action  = f"http://{esp32_ip}/action"

print("Loading YOLOv8 model...")
yolo_model = YOLO("yolov8n.pt")

# 全局变量
last_vehicle_status = False
latest_frame = None

# ================= WEB 服务器 (Flask App) =================
# 自动定位当前脚本所在的目录，确保正确映射 server 文件夹
current_dir = os.path.dirname(os.path.abspath(__file__))
server_folder = os.path.join(current_dir, "server")
app = Flask(__name__, static_folder=server_folder)

# 模拟系统状态字典
system_status = {
    "parking": [False, False, False, False, False],
    "servo": ["Closed", "Closed"],
    "ssd1306": "placeholder.png"
}

def generate_frames():
    """生成视频流的生成器，发送给前端 <img src="/video">"""
    global latest_frame
    while True:
        if latest_frame is not None:
            # 将 OpenCV 的图像矩阵编码为 JPEG 格式
            success, buffer = cv2.imencode('.jpg', latest_frame)
            if success:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            # ⚠️ 关键修复：如果暂无图像，让线程休眠，防止 CPU 100% 占用
            time.sleep(0.1)
        # 控制帧率，避免网页端渲染崩溃
        time.sleep(0.03)

@app.route("/")
def index():
    """托管本地主页"""
    return send_from_directory(server_folder, "index.html")

@app.route("/<path:file_path>")
def static_files(file_path):
    """托管前端的 CSS, JS 等静态资源"""
    return send_from_directory(server_folder, file_path)

@app.route("/video")
def video():
    """视频流 API"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """JSON 状态 API，前端可轮询此接口获取闸机状态"""
    return jsonify(system_status)

def run_web_server():
    """启动 Flask 服务的独立函数"""
    print("🌐 Web server running on http://localhost:5000")
    # debug 必须为 False，否则会在多线程中引发异常
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

# ================= AI 视觉与硬件控制循环 (AI & Hardware Loop) =================
def run_ai_loop():
    """处理图像获取、YOLO 识别与 ESP32 控制的主循环"""
    global last_vehicle_status
    global latest_frame

    print(f"🚀 System started! Connecting to: {esp32_ip}")
    print("Press 'q' in the OpenCV window to exit")

    while True:
        try:
            # 1. 从 ESP32 请求 JPEG 图像
            response = requests.get(url_capture, timeout=5)

            if response.status_code == 200:
                # 转换字节流为 Numpy 数组，供 OpenCV 解码
                img_array = np.array(bytearray(response.content), dtype=np.uint8)
                current_frame = cv2.imdecode(img_array, -1)

                if current_frame is None:
                    continue

                # 2. 执行 YOLOv8 推理
                results = yolo_model(current_frame, verbose=False)
                vehicle_detected = False

                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        confidence = float(box.conf[0])

                        # 过滤 COCO 数据集中的交通工具: 2=car, 3=motorcycle, 5=bus, 7=truck
                        if cls_id in [2, 3, 5, 7] and confidence > 0.5:
                            vehicle_detected = True
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            label_text = f"{yolo_model.names[cls_id]} {confidence:.2f}"

                            # 在图像上绘制红色边界框和标签
                            cv2.rectangle(current_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                            cv2.putText(current_frame, label_text,
                                        (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5,
                                        (0, 0, 255),
                                        2)

                # ⚠️ 关键修复：确保绘制完边界框后，再将图像传递给前端网页流
                latest_frame = current_frame.copy()
                system_status["parking"][0] = vehicle_detected

                # 3. 边缘设备通信 (Edge Device Communication)
                if vehicle_detected != last_vehicle_status:
                    if vehicle_detected:
                        print("🚨 Vehicle Detected -> Sending [Enter] command to ESP32!")
                        try:
                            # 适配 ESP32 第二版的语义化 API (?type=enter)
                            requests.get(f"{url_action}?type=enter", timeout=0.5)
                            system_status["servo"][0] = "Open"
                        except Exception as e:
                            print(f"⚠️ Failed to send Enter command: {e}")
                    else:
                        print("✅ Vehicle Cleared -> Gate is closing via ESP32 hardware timer.")
                        # ESP32 V2 代码内部自带 5 秒关闭定时器，这里只需更新本地状态
                        system_status["servo"][0] = "Closed"

                    last_vehicle_status = vehicle_detected

                # 4. PC 端本地显示 (按 Q 键退出)
                cv2.imshow("UNM Parking System - PC Server", current_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            else:
                print(f"❌ HTTP Error: {response.status_code}")

        except Exception as e:
            print(f"📡 Connection error (Retrying...): {e}")
            time.sleep(1)

    cv2.destroyAllWindows()

# ================= 主程序入口 =================
if __name__ == "__main__":
    # 使用守护线程启动 Flask 后台服务 (Daemon threads exit when the main program exits)
    web_thread = threading.Thread(target=run_web_server, daemon=True)
    web_thread.start()
    
    # 主线程必须用来跑 OpenCV 的 GUI 界面，否则容易崩溃
    run_ai_loop()
