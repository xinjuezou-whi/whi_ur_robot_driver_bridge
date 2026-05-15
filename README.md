# whi_ur_robot_driver_bridge
Launch ur_robot_driver while calling services to initialize the arm under remote control mode automatically, and advertise the service for IO manipulation. This bridge aims at a composite robotics integrated application

Features:
 - Automatically power on the controller
 - Protective stop state recovery
 - Digital output service

## Dependency
This package depends on the ur_robot_driver and externalcontrol-x.x.x.urcap. Please refer to the [official site](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) for detailed instructions

And following:
```
git clone -b ros2 https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Bringup process
![process drawio](https://github.com/xinjuezou-whi/whi_ur_robot_driver_bridge/assets/72239958/4786cfe3-d9ce-44b5-aff0-a71f11859356)

## Params
```
whi_ur_robot_driver_bridge:
  ros__parameters:
    payload_weight: 4.5 # unit Kg
    payload_to_tcp: [0.0, -0.17, 0.08]
    try_duration: 10.0 # second
    try_max_count: 20
    external_program: external_ctrl.urp
    protective_query_frequency: 20.0 # Hz
    ur_driver_patience: 55.0 # second
    moveit_cpp_state_topic: moveit_cpp_state
    debug:
      print_standby_state: false
```

If the param "protective_query_frequency" is set and greater than zero, a thread will be spawned to query the safety mode, and automatically recover UR from protective stop to normal mode.

NOTE: Before activating this mechanism, the user must ensure the consecutive path is well handled after recovering from the protective stop.

Param "motion_state_topic" is activated while the arm enters the protective stop, and the motion state message with a fault will be sent to its subscribers.

## Usage
It is recommended to extract the calibration params first, then feed it to ur_robot_driver. Please refer to [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
```
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.56.100 target_filename:=/home/nvidia/ur5e_calibration.yaml
```

> NOTE: replace ur5e with specific UR model: ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e, ur20

## Advertised service
**ur_io_request**(whi_interfaces::srv::WhiSrvIo)

Use the rosservice command line to make a quick validation:
```
ros2 service call /ur_io_request whi_interfaces/srv/WhiSrvIo "{io: {addr: <0-3>, operation: 1, level: <0/1>}}"
```

> NOTE: please refer to [ur_msgs/SetIO](https://github.com/ros-industrial/ur_msgs/blob/melodic-devel/srv/SetIO.srv) for pin number; level 1 means high and 0 means low; operation 1 means write
