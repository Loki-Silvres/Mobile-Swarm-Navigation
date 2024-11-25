import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/duck_sh/Mobile-Swarm-Navigation/install/turtlebot3_example'
