import cv2
import threading
import time
import requests
import numpy as np
from flask import Flask, Response, jsonify

# ======================================
# ESP32 摄像头端点 (建议后续将IP写在配置文件里)
# ======================================
cam_1_url = "http://10.143.39.206"   
cam_2_url = "http://10.143.39.6"   

# ======================================
# 共享内存 (帧缓冲与系统状态)
# ======================================
latest_cam_1_frame = None  
latest_cam_2_frame = None  

# 全局状态字典，用于同步给前端 Dashboard
system_status = {
    "parking": [False, False, False, False], 
    "servo": ["Closed", "Closed"]
}

# 防止重复发送开门指令的防抖时间戳
last_gate_open_time = 0

# ======================================
# 模拟 YOLO 逻辑与动作触发
# ======================================
def run_yolo_and_trigger(frame):
    global last_gate_open_time, system_status
    
    # ---- Placeholder: 假设我们做了一些图像处理 ----
    # 比如这里你未来接入 model.predict(frame)
    is_car_detected = False 
    
    # 模拟一个随机检测到车辆的场景 (用于当前测试)
    if np.random.rand() > 0.98: 
        is_car_detected = True

    current_time = time.time()
    
    # 如果检测到车，且距离上次开门超过了 6 秒 (ESP32硬件超时是5秒)
    if is_car_detected and (current_time - last_gate_open_time > 6.0):
        print("🚗 AI Detected Vehicle! Sending command to ESP32...")
        
        # 1. 异步发送开门 HTTP 请求到 ESP32
        def trigger_hardware():
            try:
                requests.get(f"{cam_1_url}/action?type=enter", timeout=2)
            except Exception as e:
                print("⚠️ Failed to open gate:", e)
                
        threading.Thread(target=trigger_hardware).start()
        
        # 2. 更新服务器内部状态，供给前端读取
        system_status["servo"][0] = "Open"
        last_gate_open_time = current_time

    # 重置前端的闸门状态 (模拟ESP32 5秒后自动关门的行为)
    if system_status["servo"][0] == "Open" and (current_time - last_gate_open_time > 5.0):
        system_status["servo"][0] = "Closed"

    return frame

# ======================================
# 抓取线程 (Grabber)
# ======================================
def grab_thread(base_url, cam_id):
    global latest_cam_1_frame, latest_cam_2_frame

    while True:
        try:
            # 组合抓图 URL
            capture_url = f"{base_url}/capture"
            r = requests.get(capture_url, timeout=1)
            
            if r.status_code == 200:
                jpg_data = np.frombuffer(r.content, dtype=np.uint8)
                frame = cv2.imdecode(jpg_data, cv2.IMREAD_COLOR)

                # 只有入口摄像头 (CAM 1) 运行 AI 检测
                if cam_id == 1:
                    frame = run_yolo_and_trigger(frame)
                    latest_cam_1_frame = frame
                else:
                    latest_cam_2_frame = frame
        except Exception as e:
            pass # 保持控制台干净，或者写日志

        time.sleep(0.05) # 稍微增加一点延时，降低ESP32负载

# ... (其余 mjpeg_stream 和 Flask 路由保持不变) ...

@app.route("/status")
def status():
    # 动态返回真实状态给前端 JS
    return jsonify(system_status)

if __name__ == "__main__":
    # ... (启动代码保持不变) ...