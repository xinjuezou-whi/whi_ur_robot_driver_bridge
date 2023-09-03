/******************************************************************
ur_robot_driver bridge to call advertised services from ur_robot_driver

Features:
- calling services
- xxx

Dependencies:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_ur_robot_driver_bridge/ur_robot_driver_bridge.h"

namespace whi_ur_robot_driver_bridge
{
    UrRobotDriverBridge::UrRobotDriverBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    void UrRobotDriverBridge::init()
    {
        // params
        // std::string planningGroup;
        // node_handle_->param("planning_group", planningGroup, std::string("whi_arm"));
    }
} // namespace whi_ur_robot_driver_bridge
