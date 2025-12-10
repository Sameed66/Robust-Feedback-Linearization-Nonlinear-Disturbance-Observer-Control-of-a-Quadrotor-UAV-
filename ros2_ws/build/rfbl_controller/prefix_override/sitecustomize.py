import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/qadeem/control/ros2_ws/install/rfbl_controller'
