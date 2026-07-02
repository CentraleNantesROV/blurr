#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from ament_index_python import get_package_share_directory
from subprocess import check_output
import shlex
import os

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor', namespace='blurr')

        self.prev = None
        self.bat = None
        self.sub = self.create_subscription(BatteryState, 'battery', self.battery_cb, rclpy.qos.qos_profile_sensor_data)
        self.img = os.path.join(get_package_share_directory('blurr'), 'config', 'logo.jpg')
        self.name = self.get_fully_qualified_name()

        self.timer = self.create_timer(5, self.notify_cb)
        self.event = None

    def battery_cb(self, msg):
        self.bat = msg
        self.bat.voltage = round(self.bat.voltage, 1)

    def notify(self, msg, level = 'normal', wait = False, options = []):

        cmd = f'notify-send "{msg}" -i "{self.img}" -a {self.name} -u {level} -p'
        if wait:
            cmd += ' -w'
        if options:
            cmd += ' -A '.join([""]+[f'"{o}"' for o in options])
        if self.event is not None:
            cmd += ' -r ' + self.event

        self.event = check_output(shlex.split(cmd)).decode().splitlines()[0]

    def notify_cb(self):

        if self.bat is None:
            return

        if self.prev is None:
            self.prev = self.bat.voltage
            self.notify(f'Battery voltage @ {self.bat.voltage}')
            return

        if self.bat.voltage != self.prev:
            self.notify(f'Battery voltage @ {self.bat.voltage}')



def main(args=None):
    rclpy.init(args=args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
