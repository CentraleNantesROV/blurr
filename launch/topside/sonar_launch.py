from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    ns = sl.declare_arg('ns', 'blurr')

    args = {'angle_step': 1,
            'angle_sector': 360,
            'frequency': 740,
            'range_max': 2.,
            'frame': 'blurr/sonar'}

    for arg, val in args.items():
        args[arg] = sl.declare_arg(arg, val)

    args.update({'connection_type': 'udp',
                 'udp_port': 9092})


    sl.node('ping360_sonar', 'ping360_node',
            parameters = args,
            namespace=ns)

    return sl.launch_description()
