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

Changelog:
2023-09-03: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include <memory>

namespace whi_ur_robot_driver_bridge
{
	class UrRobotDriverBridge
	{
    public:
        UrRobotDriverBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~UrRobotDriverBridge() = default;

    protected:
        void init();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	};
} // namespace whi_ur_robot_driver_bridge
