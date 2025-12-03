# open_manipulator_x

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
