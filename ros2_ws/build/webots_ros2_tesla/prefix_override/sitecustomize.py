import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tedee/path/ros2_ws/install/webots_ros2_tesla'
