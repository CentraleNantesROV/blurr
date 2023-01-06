from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()
    ns = sl.declare_arg('namespace', 'bluerov2')

    with sl.group(ns=ns):
        for imu in ('lsm', 'mpu'):
            sl.node('navio2_ros', 'imu', name=imu, parameters={'imu': imu})
    
    return sl.launch_description()
