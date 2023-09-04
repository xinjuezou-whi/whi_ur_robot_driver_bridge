# whi_ur_robot_driver_bridge
Launch ur_robot_driver while calling services to automatically initialize the arm under remote control mode

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
```

## Usage
```
roslaunch whi_ur_robot_driver_bridge whi_ur_robot_driver_bridge.launch kinematics_config:=${HOME}/ur10e_calibration.yaml arm_model:=ur10e robot_ip:=192.168.56.100 script_sender_port:=29999
```
