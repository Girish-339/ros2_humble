import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import cv2
import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

# ---------------- CONFIG ----------------
CAMERA_TOPICS = ['/camera1/image', '/camera2/image']
Gst.init(None)

# ---------------- STORAGE ----------------
latest_frames = [None] * len(CAMERA_TOPICS)
frame_locks = [threading.Lock() for _ in CAMERA_TOPICS]
bridge = CvBridge()

# ---------------- GStreamer pipeline for each camera ----------------
def create_gst_pipeline(cam_index):
    pipeline_desc = f"""
    appsrc name=src{cam_index} format=time is-live=true block=true max-bytes=0 caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 !
    queue max-size-buffers=2 leaky=downstream !
    videoconvert !
    videoscale !
    video/x-raw,width={width},height={height} !
    autovideoconvert !
    autovideosink sync=false
    """
    pipeline = Gst.parse_launch(pipeline_desc)
    appsrc = pipeline.get_by_name(f"src{cam_index}")
    pipeline.set_state(Gst.State.PLAYING)
    return pipeline, appsrc

pipelines = []
appsrc_list = []
for i in range(len(CAMERA_TOPICS)):
    pipe, src = create_gst_pipeline(i)
    pipelines.append(pipe)
    appsrc_list.append(src)

# ---------------- ROS2 Subscriber Callback ----------------
def camera_callback(msg, cam_index):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Store latest frame
    with frame_locks[cam_index]:
        latest_frames[cam_index] = frame

    # Push to GStreamer
    data = frame.tobytes()
    buf = Gst.Buffer.new_allocate(None, len(data), None)
    buf.fill(0, data)
    appsrc_list[cam_index].emit("push-buffer", buf)

# ---------------- ROS2 Node ----------------
class MultiCamSubscriber(Node):
    def __init__(self):
        super().__init__('multi_cam_subscriber')
        for idx, topic in enumerate(CAMERA_TOPICS):
            self.create_subscription(
                Image,
                topic,
                lambda msg, i=idx: camera_callback(msg, i),
                10
            )

# ---------------- THREAD TO RUN ROS2 ----------------
def ros2_thread():
    rclpy.init()
    node = MultiCamSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=ros2_thread, daemon=True).start()

# ---------------- DISPLAY ----------------
try:
    while True:
        for idx, frame in enumerate(latest_frames):
            with frame_locks[idx]:
                if frame is not None:
                    cv2.imshow(f"Camera {idx+1}", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    for p in pipelines:
        p.set_state(Gst.State.NULL)
