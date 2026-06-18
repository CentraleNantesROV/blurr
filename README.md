ROS package for high-level ROS launch of the BlueOS stack.

The main launch file runs :

    - mavros
    - v4l2_camera
    - the `pwm` node from this package


## `pwm`

This node converts thrusts, lights and camera tilt to pwm outputs.

parameters: `use_wrench_input`
    - if True, subscribe to `Wrench` on `cmd_wrench` if the ROV is configured for manual control
    - if False, subscribe to `JointState` on `cmd_thrust` if the ROV is configured for thruster control


# Links


https://ardupilot.org/sub/docs/parameters.html

https://github.com/Robotic-Decision-Making-Lab/underwater_docking/blob/72d33ec6e4a3158b50b05181959ef091b909aeb1/docking_control/src/mission_control.py#L200

```python
# Initialize arm/disarm service
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        # Enable/Disable RC Passthrough Mode
        self.rc_passthrough_srv = rospy.ServiceProxy(
            "/blue/cmd/enable_passthrough", SetBool
        )
```
