#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from math import pi
from geometry_msgs.msg import Wrench
from mavros_msgs.srv import CommandBool

class JoyToWrenchNode(Node):
    def __init__(self):
        super().__init__('teleop')

        # DOF limits: parameters are flattened (e.g. 'surge.max', 'surge.min')
        default_limit = {'max': 20.0, 'min': -20.0}
        self.limits = {}
        for dof in ['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw']:
            for bound in ('min', 'max'):
                key = f'{dof}.{bound}'
                self.limits[key] = self.declare_parameter(key, default_limit[bound]).value

        # joystick mapping (flattened under 'joy.<name>')
        default_joy_map = {
            'surge': 1,
            'sway': 0,
            'heave': 4,
            'roll': 6,
            'pitch': 1000,
            'yaw': 3,
            'tilt': -7,
            'light': 3,
            'arm': 8,
            'scale_up': 1,
            'scale_down': 2,
            'tilt_reset': 6
        }

        self.joy_map = {}
        for dof, default_idx in default_joy_map.items():
            idx = self.declare_parameter(f'joy.{dof}', default_idx).value
            coef = -1 if idx < 0 else 1
            idx = int(abs(idx))
            self.joy_map[dof] = (idx, coef)


        self.wrench_pub = self.create_publisher(Wrench, 'cmd_wrench', 1)
        self.tilt_pub = self.create_publisher(Float32, 'cmd_tilt', 1)
        self.light_pub = self.create_publisher(Float32, 'cmd_light', 1)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.arm_client = self.create_client(CommandBool, 'cmd/arming')

        # state for incremental controls
        self.tilt = Float32()
        self.light = Float32()
        self.arm = CommandBool.Request()
        self.arm.value = True
        self.scale = 1.

        self.joy = None
        self.timer = self.create_timer(0.1, self.publish)

    def scale_axis_to_range(self, axis_val, min_v, max_v):
        # axis_val in [-1, 1] -> scale to [min_v, max_v]
        return axis_val * (max_v - min_v) / 2.0 + (max_v + min_v) / 2.0

    def joy_cb(self, msg: Joy):

        if self.joy is not None:

            def has_changed(dof):
                idx = self.joy_map[dof][0]
                val = msg.buttons[idx]
                prev_val = self.joy.buttons[idx]
                return val == 1 and prev_val == 0

            # detect press of light button
            if has_changed('light'):
                self.light.data = 1.-self.light.data

            if has_changed('tilt_reset'):
                self.tilt.data = 0.

            # detect press of arm button
            if has_changed('arm'):
                self.arm.value = not self.arm.value
                self.arm_client.call_async(self.arm)
                if self.arm.value:
                    self.get_logger().info('Arming')
                else:
                    self.get_logger().info('Dearming')

            # detect scale up
            if has_changed('scale_up'):
                self.scale = min(1., max(0., self.scale + 0.05))
                self.get_logger().info(f'Scale @ {100*self.scale:.0f} %')
            elif has_changed('scale_down'):
                self.scale = min(1., max(0., self.scale - 0.05))
                self.get_logger().info(f'Scale @ {100*self.scale:.0f} %')

        self.joy = msg

    def publish(self):

        if self.joy is None:
            return

        # wrench part
        wrench = Wrench()
        for dof in ('surge','sway','heave','roll','pitch','yaw'):
            idx, coef = self.joy_map[dof]
            if idx >= len(self.joy.axes):
                continue
            scaled = self.scale_axis_to_range(coef*self.joy.axes[idx],
                                              self.limits[f'{dof}.min'],
                                              self.limits[f'{dof}.max'])
            scaled *= self.scale

            if dof == 'surge':
                wrench.force.x = scaled
            elif dof == 'sway':
                wrench.force.y = scaled
            elif dof == 'heave':
                wrench.force.z = scaled
            elif dof == 'roll':
                wrench.torque.x = scaled
            elif dof == 'pitch':
                wrench.torque.y = scaled
            elif dof == 'yaw':
                wrench.torque.z = scaled

        self.wrench_pub.publish(wrench)

        # tilt with internal state
        idx, coef = self.joy_map['tilt']

        # small incremental step per message
        inc = self.scale_axis_to_range(coef*self.joy.axes[idx], -0.1, 0.1)
        self.tilt.data = min(pi/4, max(-pi/4, self.tilt.data + inc))
        if abs(self.tilt.data) < 0.015:
            self.tilt.data = 0.
        self.tilt_pub.publish(self.tilt)

        self.light_pub.publish(self.light)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToWrenchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
