# whi_ur_robot_driver_bridge
Launch ur_robot_driver while calling services to initialize the arm under remote control mode automatically

## Dependency
This package depends on the ur_robot_driver and externalcontrol-x.x.x.urcap. Please refer to the [official site](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) for detailed instructions

## Bringup process
![process drawio](https://github.com/xinjuezou-whi/whi_ur_robot_driver_bridge/assets/72239958/4786cfe3-d9ce-44b5-aff0-a71f11859356)

## Params
```
whi_ur_robot_driver_bridge:
  try_duration: 2 # second
  try_max_count: 10
  external_program: external_ctrl.urp
  protective_query_frequency: 5.0 #Hz
  motion_state_topic: arm_motion_state
```

If the param "protective_query_frequency" is set and greater than zero, a thread will be spawned to query the safety mode, and automatically recover UR from protective stop to normal mode.

NOTE: Before activating this mechanism, it is the user's responsibility to ensure the consecutive path is well handled after recovering from the protective stop.

Param "motion_state_topic" is activated while the arm enters the protective stop, and the motion state message with fault will be sent to its subscribers.

## Usage
It is recommended to extract the calibration params first, then feed it to ur_robot_driver. Please refer to [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
```
roslaunch whi_ur_robot_driver_bridge whi_ur_robot_driver_bridge.launch kinematics_config:=${HOME}/ur10e_calibration.yaml arm_model:=ur10e robot_ip:=192.168.56.100 script_sender_port:=29999
```

> NOTE: replace ur10e with specific UR model: ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e, ur20
