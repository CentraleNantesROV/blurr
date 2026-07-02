from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    rviz = sl.declare_arg('rviz', False)

    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': 'blurr',
                                 'use_sim_time': False,
                                 'jsp': False})

    if sl.arg('rviz'):
        # check IMU pose
        sl.node('localization_msgs_tools', 'pose_to_tf',
                parameters = {'topic': '/blurr/imu/data',
                              'parent_frame': 'blurr/imu',
                              'child_frame': 'world',
                              'inverse': True})
        sl.rviz(sl.find('blurr', 'imu.rviz'))

    return sl.launch_description()
