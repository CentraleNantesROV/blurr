from simple_launch import SimpleLauncher
from mavros_msgs.srv import SetMode

def generate_launch_description():

    sl = SimpleLauncher()
    sl.declare_arg("fcu_url", "tcp://0.0.0.0:5777@")
    sl.declare_arg("gcs_url", "")
    sl.declare_arg("tgt_system", "1")
    sl.declare_arg("tgt_component", "1")
    sl.declare_arg("log_output", "screen")
    sl.declare_arg("fcu_protocol", "v2.0")
    sl.declare_arg("respawn_mavros", "false")
    sl.declare_arg("namespace", "blurr")

    mavros_args = sl.arg_map("fcu_url", "gcs_url", "tgt_system", "tgt_component",
                          "log_output", "fcu_protocol", "respawn_mavros", "namespace")
    mavros_args.update({"pluginlists_yaml": sl.find('blurr', 'apm_pluginlist.yaml'),
                     "config_yaml": sl.find('blurr', 'apm_config.yaml')})

    sl.include('mavros', 'node.launch',
            launch_arguments=mavros_args)

    with sl.group(ns = sl.arg('namespace')):

        # RC Override for wrench + light / tilt
        sl.node('blurr', 'pwm',
                parameters = [sl.find('blurr', 'bounds.yaml')])

        # arm and go manual
        sl.call_service('cmd/arming', {'value': True})
        sl.call_service('set_mode', {'base_mode': SetMode.Request.MAV_MODE_MANUAL_ARMED})

    return sl.launch_description()
