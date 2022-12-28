# open_manipulator_x


To turn off the torque of manipulator joints you have to manually set hardware controller to inactive state: 
```
ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "{name: 'manipulator', target_state: {id: 0, label: 'inactive'}}"
```

To activate it again use the same service, but change label to `active`.