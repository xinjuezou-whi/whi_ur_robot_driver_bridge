# whi_ur_robot_driver_bridge
launch ur_robot_driver while calling services to automatically initialize the arm under remote control mode

## Bringup process
![process drawio](https://github.com/xinjuezou-whi/whi_ur_robot_driver_bridge/assets/72239958/4786cfe3-d9ce-44b5-aff0-a71f11859356)

## Params
```
whi_ur_robot_driver_bridge:
  try_duration: 2 # second
  try_max_count: 10
  external_program: external_ctrl.urp
```
