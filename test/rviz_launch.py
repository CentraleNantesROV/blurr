from simple_launch import SimpleLauncher
from os import path
from math import pi

sl = SimpleLauncher()
sl.declare_arg('imu','mpu')


def launch_setup():

    imu = sl.arg('imu')
    base_dir = path.dirname(__file__)

    if imu == 'ekf':
        # fused imu, description already here
        sl.rviz(path.join(base_dir, 'ekf.rviz'))
    else:
        # basic imu test
        sl.rviz(path.join(base_dir, 'imu.rviz'))

        sl.include('bluerov2_description', 'state_publisher_launch.py')

        sl.node('pose_to_tf', 'pose_to_tf', parameters={'topic': imu})

        sl.node('tf2_ros', 'static_transform_publisher',
                name = imu + '_tf',
                arguments = ['--frame-id', imu] + f'--child-frame-id bluerov2/base_link --y -0.1 --yaw {pi/2}'.split())
    
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
