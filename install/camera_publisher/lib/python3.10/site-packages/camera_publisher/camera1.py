import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import urllib.parse

class VideoFilePublisher(Node):
    def __init__(self):
        super().__init__('camera_file_publisher')

        self.publisher = self.create_publisher(Image, '/camera1/image', 10)
        self.bridge = CvBridge()

        # RTSP URL (password-safe)
        password = urllib.parse.quote('abcd@1234')
        url = f'rtsp://admin:{password}@192.168.1.8:554'

        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Cannot open RTSP stream")

        # Target resolution (smaller than screen)
        self.target_width = 640
        self.target_height = 360

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.fps <= 0 or self.fps > 60:
            self.fps = 30

        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info("✅ RTSP CCTV → ROS Image (resized) started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ RTSP frame not received")
            return

        # Resize frame
        frame = cv2.resize(
            frame,
            (self.target_width, self.target_height),
            interpolation=cv2.INTER_LINEAR
        )

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = VideoFilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
