import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/hri-project/ros2_ws/install/robot_control_architecture_pkg'
