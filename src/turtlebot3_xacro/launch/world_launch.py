from simple_launch import SimpleLauncher

sl = SimpleLauncher()
sl.declare_arg('gui', default_value=True)


def launch_setup():

    gz_args = f"-r {sl.find('turtlebot3_xacro', 'turtlebot3_world.sdf')}"
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(gz_args)
    sl.create_gz_clock_bridge()

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
