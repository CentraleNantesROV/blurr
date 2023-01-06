from simple_launch import SimpleLauncher
from os import path


def generate_launch_description():
    
    sl = SimpleLauncher()

    base_dir = path.abspath(path.dirname(__file__))

    sl.include('bluerov2_description', 'state_publisher_launch.py')

    sl.node('robot_localization', 'ekf_node',
            parameters=[base_dir + '/' + 'ekf.yaml'])
    
    return sl.launch_description()
