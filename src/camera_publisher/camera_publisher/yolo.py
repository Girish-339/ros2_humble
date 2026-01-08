import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import cv2
import time
import json
import requests

from ultralytics import YOLO

# ================= CONFIG =================
CAMERA_TOPICS = ["/camera1/image", "/camera2/image"]

FASTAPI_URLS = {
    0: "http://localhost:8002/update/1",
    1: "http://localhost:8002/update/2",
}

MODEL_PATH = "yolov8n.pt"
CONFIDENCE = 0.3
FILTER_CLASSES = None   # Example ["person"]

# ================= GLOBALS =================
bridge = CvBridge()
latest_frames = [None, None]
frame_locks = [threading.Lock(), threading.Lock()]

print("🚀 Loading YOLO model...")
model = YOLO(MODEL_PATH)
print("✅ YOLO loaded")

# ================= CALLBACK =================
def camera_callback(msg, cam_index):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    results = model(frame, conf=CONFIDENCE, verbose=False)
    detections = []

    if results and results[0].boxes is not None:
        for box in results[0].boxes:
            cls_id = int(box.cls)
            cls_name = model.names[cls_id]
            conf = float(box.conf)

            if FILTER_CLASSES and cls_name not in FILTER_CLASSES:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            detections.append({
                "class": cls_name,
                "confidence": round(conf, 3),
                "bbox": [x1, y1, x2, y2],
                "timestamp": time.time()
            })

            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(
                frame,
                f"{cls_name} {conf:.2f}",
                (x1, y1-6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0,255,0),
                2
            )

    # Save local copy
    with frame_locks[cam_index]:
        latest_frames[cam_index] = frame.copy()

    # ================= SEND TO FASTAPI =================
    try:
        _, jpg = cv2.imencode(".jpg", frame)

        response = requests.post(
            FASTAPI_URLS[cam_index],
            files={
                "frame": ("frame.jpg", jpg.tobytes(), "image/jpeg")
            },
            data={
                "detections": json.dumps(detections)
            },
            timeout=0.5
        )

        if response.status_code != 200:
            print(f"❌ FastAPI {response.status_code}: {response.text}")

    except Exception as e:
        print("⚠️ FastAPI error:", e)

    if detections:
        print(f"[Camera {cam_index+1}]",
              [d["class"] for d in detections])

# ================= ROS2 NODE =================
class MultiCamNode(Node):
    def __init__(self):
        super().__init__("multi_cam_yolo")

        for idx, topic in enumerate(CAMERA_TOPICS):
            self.create_subscription(
                Image,
                topic,
                lambda msg, i=idx: camera_callback(msg, i),
                10
            )

# ================= ROS THREAD =================
def ros_thread():
    rclpy.init()
    node = MultiCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=ros_thread, daemon=True).start()

# ================= LOCAL VIEW =================
try:
    while True:
        for i in range(2):
            with frame_locks[i]:
                if latest_frames[i] is not None:
                    cv2.imshow(f"Camera {i+1}", latest_frames[i])
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
