import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/franfuentes/ros2_sjtu_ws/src/install/sjtu_drone_control'
