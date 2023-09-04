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
        ~UrRobotDriverBridge();

    protected:
        void init();
        void beStandby();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::ServiceClient> client_power_off_{ nullptr };
        int try_duration_{ 2 }; // second
        int try_max_count_{ 10 };
        std::string external_program_{ "external_ctrl.urp" };
	};
} // namespace whi_ur_robot_driver_bridge
