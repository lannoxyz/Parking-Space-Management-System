import cv2
import numpy as np
import requests
from ultralytics import YOLO
import time

# ================= CONFIGURATION =================
# ⚠️⚠️⚠️ Please replace this with the IP address printed in your Serial Monitor ⚠️⚠️⚠️
ESP32_IP = "192.168.0.25" 

URL_CAPTURE = f"http://{ESP32_IP}/capture"
URL_ACTION  = f"http://{ESP32_IP}/action"

# Load the model
print("Loading YOLOv8 model...")
model = YOLO("yolov8n.pt") 

last_vehicle_status = False # Track the previous state to avoid redundant requests

print(f"🚀 System started! Connecting to: {ESP32_IP}")
print("Press 'q' to exit")

while True:
    try:
        # 1. Request image from ESP32
        response = requests.get(URL_CAPTURE, timeout=5)
        
        if response.status_code == 200:
            # Decode the image
            img_array = np.array(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, -1)
            
            if frame is None: 
                continue

            # 2. AI Inference
            results = model(frame, verbose=False)
            vehicle_detected = False
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # COCO IDs: 2(car), 3(motorcycle), 5(bus), 7(truck)
                    if cls_id in [2, 3, 5, 7] and conf > 0.5:
                        vehicle_detected = True
                        
                        # Draw bounding boxes (Red)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        label = f"{model.names[cls_id]} {conf:.2f}"
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                        cv2.putText(frame, label, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 3. Send command to ESP32 (Only on state change)
            if vehicle_detected != last_vehicle_status:
                if vehicle_detected:
                    print("🚨 Vehicle Detected -> Turn Light ON!")
                    try: 
                        requests.get(f"{URL_ACTION}?val=1", timeout=0.5)
                    except: 
                        pass
                else:
                    print("✅ Vehicle Cleared -> Turn Light OFF")
                    try: 
                        requests.get(f"{URL_ACTION}?val=0", timeout=0.5)
                    except: 
                        pass
                
                last_vehicle_status = vehicle_detected

            # Display the result
            cv2.imshow("ESP32 AI Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("❌ Failed to fetch image (HTTP Error)")
            
    except Exception as e:
        print(f"Connection error (Retrying...): {e}")
        time.sleep(1)

cv2.destroyAllWindows()
