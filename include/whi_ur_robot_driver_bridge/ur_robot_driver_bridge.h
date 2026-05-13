/******************************************************************
ur_robot_driver bridge to call advertised services from ur_robot_driver

Features:
- calling services
- xxx

Dependencies:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-09-03: Initial version
2026-04-27: Migrated from ROS 1
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/srv/whi_srv_io.hpp"
#include "whi_interfaces/msg/whi_motion_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace whi_ur_robot_driver_bridge
{
	class UrRobotDriverBridge
	{
    public:
        UrRobotDriverBridge(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~UrRobotDriverBridge();

    protected:
        void init();
        void beStandby();
        void threadSafty();
        std::string getLoadedProgram();
        int requestLoadProgram();
        int deactiveRunningProgram();
        int requestPlay();
        int powerOn();
        int powerOff();
        int releaseBrake();
        int closePopups();
        int isProtective(bool& IsProtective);
        int recoverFromProtective();
        bool isInRemote();
        bool isProgramRunning();
        int disconnect();
        int reconnect();
        bool setPayload();
        bool handBackControl();
        void onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> Response);
        void onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> Response);
        void callbackMoveitCppState(const std_msgs::msg::Bool::SharedPtr Msg);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        int try_duration_{ 2000 }; // millisecond
        int try_max_count_{ 10 };
        std::string external_program_{ "external_ctrl.urp" };
        std::mutex mtx_;
	    std::thread th_safty_;
        std::condition_variable cv_;
        bool standby_{ false };
        std::atomic_bool terminated_{ false };
        int safty_query_duration_{ 0 };
        rclcpp::Publisher<whi_interfaces::msg::WhiMotionState>::SharedPtr pub_motion_state_{ nullptr };
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_moveit_cpp_state_{ nullptr };
        std::string prefix_dashboard_{ "dashboard_client/" };
        rclcpp::Service<whi_interfaces::srv::WhiSrvIo>::SharedPtr server_io_{ nullptr };
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_ready_{ nullptr };
        bool moveit_cpp_ready_{ false };
        enum Res { RES_SUCCEED = 0, RES_FAILED_EXECUTE };
	};
} // namespace whi_ur_robot_driver_bridge
