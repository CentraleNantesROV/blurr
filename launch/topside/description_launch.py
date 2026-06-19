from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': 'blurr',
                                 'use_sim_time': False,
                                 'jsp': False})
    return sl.launch_description()
