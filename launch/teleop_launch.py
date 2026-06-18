from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    with sl.group(ns = 'blurr'):

        sl.node('joy', 'joy_node')

        sl.node('blurr', 'teleop.py') #, parameters = [sl.find('blurr', 'bounds.yaml')])

    return sl.launch_description()
