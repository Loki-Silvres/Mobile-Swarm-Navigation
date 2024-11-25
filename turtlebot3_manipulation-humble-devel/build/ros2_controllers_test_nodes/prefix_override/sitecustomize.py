import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/duck_sh/turtlebot3_manipulation-humble-devel/install/ros2_controllers_test_nodes'
