from simple_launch import SimpleLauncher
from os import path


def generate_launch_description():
    
    sl = SimpleLauncher()
    ns = sl.declare_arg('namespace', 'bluerov2')

    base_dir = path.abspath(path.dirname(__file__))

    sl.include('bluerov2_description', 'state_publisher_launch.py')

    with sl.group(ns=ns):
        sl.node('robot_localization', 'ekf_node',
            parameters=[base_dir + '/' + 'ekf.yaml'])

    base_link = ns + '/base_link'
    odom = ns + '/odom'
    sl.node('tf2_ros', 'static_transform_publisher',
                name = 'odom',
                arguments = ['--frame-id', odom, '--child-frame-id', base_link])
    
    return sl.launch_description()
