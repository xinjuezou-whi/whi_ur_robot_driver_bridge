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
2026-05-15: Switch to lifecycle node and add bond connection for lifecycle manager
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/srv/whi_srv_io.hpp"
#include "whi_interfaces/msg/whi_motion_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bondcpp/bond.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace whi_ur_robot_driver_bridge
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

	class UrRobotDriverBridge : public rclcpp_lifecycle::LifecycleNode
	{
    public:
        UrRobotDriverBridge(const std::string& NodeName = "whi_ur_robot_driver_bridge",
            const rclcpp::NodeOptions& Options = rclcpp::NodeOptions());
        ~UrRobotDriverBridge();

    public:
        // Create bond connection for nav2 lifecycle manager
        void createBond();
        // Destroy bond connection for nav2 lifecycle manager
        void destroyBond();
        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

    protected:
        int checkingConnection();
        void beStandby();
        void threadSafty();
        std::string getLoadedProgram();
        int requestLoadProgram();
        int deactiveRunningProgram();
        int requestPlay();
        int powerOn();
        int powerOff(bool Sync = true);
        int releaseBrake();
        int closePopups();
        int isProtective(bool& IsProtective, bool Sync = true);
        int recoverFromProtective();
        int stopProgram(bool Sync = true);
        bool isInRemote();
        bool isProgramRunning();
        int disconnect(bool Sync = true);
        int reconnect();
        bool setPayload();
        bool handBackControl();
        void onServiceIo(std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvIo>> Service,
            const std::shared_ptr<rmw_request_id_t> RequestHeader,
            const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request); // nested service call type
        void onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> Response);
        void callbackMoveitCppState(const std_msgs::msg::Bool::SharedPtr Msg);

    protected:
        double payload_weight_{ 4.5 };
        std::vector<double> payload_to_tcp_;
        int try_duration_{ 20000 }; // millisecond
        int try_max_count_{ 10 };
        std::string external_program_{ "external_ctrl.urp" };
        std::mutex mtx_;
	    std::thread th_safty_;
        std::condition_variable cv_;
        bool standby_{ false };
        std::atomic_bool terminated_{ false };
        int safty_query_duration_{ 0 };
        rclcpp_lifecycle::LifecyclePublisher<whi_interfaces::msg::WhiMotionState>::SharedPtr pub_motion_state_{ nullptr };
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_moveit_cpp_state_{ nullptr };
        std::string prefix_dashboard_{ "dashboard_client/" };
        rclcpp::Service<whi_interfaces::srv::WhiSrvIo>::SharedPtr server_io_{ nullptr };
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_ready_{ nullptr };
        bool moveit_cpp_ready_{ false };
        enum Res { RES_SUCCEED = 0, RES_FAILED_EXECUTE };
        bool debug_print_standby_state_{ false };
        std::string robot_ip_;
        int ur_driver_patience_{ 20000 }; // millisecond
        enum ConnectionTried { ONE_SHOT = 0, REPEATED, MAX_TRIED };

        // Connection to tell that server is still up
        std::shared_ptr<bond::Bond> bond_{nullptr};
	};
} // namespace whi_ur_robot_driver_bridge
