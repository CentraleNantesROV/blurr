ROS package for high-level ROS launch of the BlueOS stack.

# Overall procedure

## Connect to the BlueROV and run the stack inside the docker

```bash
# log in BlueOS
ssh pi@blueos.local
# log in the ROS 2 docker
./ros2docker.bash
cd persistent_ws
# source the ROS 2 workspaces
source rmt.bash
# run the stack
ros2 launch blurr bringup.launch
```

The main launch file runs :

- mavros
- v4l2_camera
- the `pwm` node from this package


### `pwm`

This node converts thrusts, lights and camera tilt to pwm outputs.

parameters: `use_wrench_input`

- if True, subscribe to `Wrench` on `cmd_wrench` if the ROV is configured for manual control
- if False, subscribe to `JointState` on `cmd_thrust` if the ROV is configured for thruster control

## From the topside computer

Either publish / subscribe directly to topics, and / or use the launch files from the `topside` folder:

- `description_launch.py`: spawns a `robot_state_publisher`
- `sonar_launch.py`: runs the [ping360](https://github.com/CentraleNantesRobotics/ping360_sonar) node
- `teleop_launch.py`: runs teleop + joy nodes

# Some links


https://ardupilot.org/sub/docs/parameters.html

https://github.com/Robotic-Decision-Making-Lab/underwater_docking/blob/72d33ec6e4a3158b50b05181959ef091b909aeb1/docking_control/src/mission_control.py#L200

