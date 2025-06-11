# open_manipulator_x

In this repo, you can find the integration of OpenManipulatorX with `ros2_control`. Based on the [code provided by Robotis](https://github.com/ROBOTIS-GIT/open_manipulator), updated to work with the Humble distribution of ROS 2. You can use it by including `open_manipulator_x_macro` in the URDF of your robot, for example:

```xml
<xacro:include
    filename="$(find open_manipulator_x_description)/urdf/open_manipulator_x_macro.urdf.xacro"
    ns="manipulator" />
  <xacro:manipulator.open_manipulator_x parent_link="cover_link"
    xyz="-0.1 0.0 0.0"
    rpy="0.0 0.0 0.0"
    use_sim="False"
    collision_enabled="True"
    usb_port="/dev/ttyUSB0"
    baud_rate="1000000"
    joint1_limit_min="-2.356"
    joint1_limit_max="5.934" />
```

Parameters description:

* `parent_link` link to which manipulator will be attached (with transform specified by `xyz` and `rpy` parameters)
* `use_sim` - whether simulation ros2_control plugin should be included instead of real hardware one (currently only `Gazebo` is supported).
* `collision_enabled` - due to lack of dedicated collision meshes of open_manipulator_x, visual ones are also used for collisions. This can result in a drop in simulation performance, you can opt to disable collisions in this case.
* `usb_port` in case of using hardware, name of USB port where manipulator is connected (e.g. `/dev/ttyUSB0`).
* `baud_rate` in case of using hardware, a baud rate of communication with servos.

Additionally, you have to add an appropriate controller, an example is provided in the `open_manipulator_x_hardware/config` directory.

## Usage

Before activating torque, the current position of the manipulator is read, then it is set as a goal position - the manipulator stays in the position that it was activated.

### Turning off torque

One useful thing that can be a little tricky is turning off the torque of the manipulator's joints. To do it, you have to manually set the hardware controller to an inactive state:

```bash
ros2 service call /controller_manager/set_hardware_component_state \
  controller_manager_msgs/srv/SetHardwareComponentState \
  "{name: 'manipulator', target_state: {id: 0, label: 'inactive'}}"
```

To activate it again use the same service, but change the label to `active`.
