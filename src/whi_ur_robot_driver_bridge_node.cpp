/******************************************************************
node to bridge ur_robot_driver

Features:
- service calling
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-09-03: Initial version
2026-04-27: Migrated from ROS 1
2026-05-15: Switch to lifecycle node and add bond connection for lifecycle manager
2026-xx-xx: xxx
******************************************************************/
#include "whi_ur_robot_driver_bridge/ur_robot_driver_bridge.h"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <signal.h>
#include <functional>

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI UR robot driver bridge VERSION 02.10.2" << std::endl;
	std::cout << "Copyright © 2023-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
	rclcpp::init(argc, argv);

	/// node logic
	const std::string nodeName("whi_ur_robot_driver_bridge"); 
	auto instance = std::make_shared<whi_ur_robot_driver_bridge::UrRobotDriverBridge>(nodeName);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		instance = nullptr;

		// all the default sigint handler does is call shutdown()
		rclcpp::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(instance->get_node_base_interface());
    executor->spin();  // blocking until shutdown
#else
    rclcpp::spin(instance->get_node_base_interface());
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
