from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time=False)
sl.declare_arg('name', 'waffle1')


def generate_launch_description():

    with sl.group(ns = sl.arg('name')):
        sl.node('slider_publisher', arguments = sl.find('turtlebot3_xacro', 'Twist.yaml'))
    return sl.launch_description()
