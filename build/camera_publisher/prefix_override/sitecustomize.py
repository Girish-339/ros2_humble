import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/girish/ros2_multi_camera_ws/install/camera_publisher'
