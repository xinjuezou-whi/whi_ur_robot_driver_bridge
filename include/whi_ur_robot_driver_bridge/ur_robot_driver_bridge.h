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
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/WhiSrvIo.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

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
        bool onServiceIo(whi_interfaces::WhiSrvIo::Request& Request, whi_interfaces::WhiSrvIo::Response& Response);
        bool onServiceReady(std_srvs::Trigger::Request& Request, std_srvs::Trigger::Response& Response);
        void callbackMotionState(const std_msgs::Bool::ConstPtr& Msg);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::shared_ptr<ros::NodeHandle> node_handle_ns_free_{ nullptr };
        int try_duration_{ 2 }; // second
        int try_max_count_{ 10 };
        std::string external_program_{ "external_ctrl.urp" };
        std::mutex mtx_;
	    std::thread th_safty_;
        std::condition_variable cv_;
        bool standby_{ false };
        std::atomic_bool terminated_{ false };
        int safty_query_duration_{ 0 };
        std::unique_ptr<ros::Publisher> pub_motion_state_{ nullptr };
        std::unique_ptr<ros::Subscriber> sub_moveit_cpp_state_{ nullptr };
        std::string service_prefix_{ "ur_hardware_interface/" };
        std::string prefix_dashboard_{ "dashboard/" };
        std::unique_ptr<ros::ServiceServer> server_io_{ nullptr };
        std::unique_ptr<ros::ServiceServer> server_ready_{ nullptr };
        bool moveit_cpp_ready_{ false };
        enum Res { RES_SUCCEED = 0, RES_FAILED_EXECUTE, RES_FAILED_CALL };
	};
} // namespace whi_ur_robot_driver_bridge
