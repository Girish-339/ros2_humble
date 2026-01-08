from fastapi import FastAPI, UploadFile, File, Form, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
import cv2
import numpy as np
import json
import asyncio
from typing import Dict, List

app = FastAPI()

# ---------------- STORAGE ----------------
frames: Dict[int, np.ndarray] = {}
latest_detections: Dict[int, list] = {}
history: Dict[int, List[dict]] = {}
clients: Dict[int, List[WebSocket]] = {}

# ---------------- MJPEG STREAM ----------------
async def mjpeg_stream(cam_id: int):
    try:
        while True:
            if cam_id in frames:
                frame = frames[cam_id]
                _, jpg = cv2.imencode(".jpg", frame)

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    jpg.tobytes() +
                    b"\r\n"
                )

            await asyncio.sleep(0.03)  # ✅ MUST be awaited

    except asyncio.CancelledError:
        print(f"🛑 MJPEG stream cancelled for camera {cam_id}")
        raise


@app.get("/video/{cam_id}")
def video(cam_id: int):
    return StreamingResponse(
        mjpeg_stream(cam_id),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

# ---------------- UPDATE FROM YOLO ----------------
@app.post("/update/{cam_id}")
async def update(
    cam_id: int,
    frame: UploadFile = File(...),
    detections: str = Form(...)
):
    img_bytes = await frame.read()
    np_img = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

    frames[cam_id] = img

    dets = json.loads(detections)
    latest_detections[cam_id] = dets
    history.setdefault(cam_id, []).extend(dets)

    # Push to WebSocket clients
    if cam_id in clients:
        dead_clients = []
        for ws in clients[cam_id]:
            try:
                await ws.send_json({
                    "latest": dets,
                    "history": history[cam_id][-50:]
                })
            except Exception:
                dead_clients.append(ws)

        # Remove disconnected clients
        for ws in dead_clients:
            clients[cam_id].remove(ws)

    return {"status": "ok"}

# ---------------- WEBSOCKET ----------------
@app.websocket("/ws/detections/{cam_id}")
async def ws_detections(ws: WebSocket, cam_id: int):
    await ws.accept()
    clients.setdefault(cam_id, []).append(ws)

    try:
        while True:
            await asyncio.sleep(10)  # keep alive

    except WebSocketDisconnect:
        print(f"🔌 WebSocket disconnected (camera {cam_id})")

    except asyncio.CancelledError:
        print(f"🛑 WebSocket cancelled (camera {cam_id})")

    finally:
        if cam_id in clients and ws in clients[cam_id]:
            clients[cam_id].remove(ws)

