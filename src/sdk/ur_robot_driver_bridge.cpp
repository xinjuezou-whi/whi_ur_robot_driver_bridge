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

******************************************************************/
#include "whi_ur_robot_driver_bridge/ur_robot_driver_bridge.h"
#include "whi_interfaces/msg/whi_motion_state.hpp"

#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>
#include <ur_dashboard_msgs/srv/get_program_state.hpp>
#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include <ur_dashboard_msgs/srv/get_loaded_program.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>
#include <ur_dashboard_msgs/srv/is_in_remote_control.hpp>
#include <ur_msgs/srv/set_payload.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <thread>

namespace whi_ur_robot_driver_bridge
{
    UrRobotDriverBridge::UrRobotDriverBridge(const std::string& NodeName/* = "whi_ur_robot_driver_bridge"*/,
        const rclcpp::NodeOptions& Options/* = rclcpp::NodeOptions()*/)
        : rclcpp_lifecycle::LifecycleNode(NodeName, "", Options)
    {
        // params
        declare_parameter("payload_weight", 4.5);
        declare_parameter("payload_to_tcp", std::vector<double>{});
        declare_parameter("try_duration", 5.0);
        declare_parameter("try_max_count", 20);
        declare_parameter("external_program", std::string("external_ctrl.urp"));
        declare_parameter("protective_query_frequency", 20.0);
        declare_parameter("ur_driver_patience", 20.0);
        declare_parameter("moveit_cpp_sate_topic", std::string("moveit_cpp_state"));
        declare_parameter("debug.print_standby_state", false);
        declare_parameter("robot_ip", std::string());
    }

    UrRobotDriverBridge::~UrRobotDriverBridge()
    {
        stopProgram(false);
        powerOff(false);
        disconnect(false);

        terminated_.store(true);
	    if (th_safty_.joinable())
	    {
		    th_safty_.join();
	    }
    }

    void UrRobotDriverBridge::createBond()
    {
        RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", get_name());

        bond_ = std::make_shared<bond::Bond>(std::string("bond"), get_name(), shared_from_this());

        bond_->setHeartbeatPeriod(0.1);
        bond_->setHeartbeatTimeout(4.0);
        bond_->start();
    }

    void UrRobotDriverBridge::destroyBond()
    {
        RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", get_name());

        if (bond_)
        {
            bond_.reset();
        }
    }

    CallbackReturn UrRobotDriverBridge::on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        // params
        payload_weight_ = get_parameter("payload_weight").as_double();
        payload_to_tcp_ = get_parameter("payload_to_tcp").as_double_array();
        if (payload_to_tcp_.empty())
        {
            payload_to_tcp_.resize(3, 0.0);
        }
        try_duration_ = int(1000.0 * get_parameter("try_duration").as_double());
        try_max_count_ = get_parameter("try_max_count").as_int();
        external_program_ = get_parameter("external_program").as_string();
        double frequency = get_parameter("protective_query_frequency").as_double();
        if (frequency > 1e-5)
        {
            safty_query_duration_ = int(1000.0 / frequency);
        }
        ur_driver_patience_ = int(1000.0 * get_parameter("ur_driver_patience").as_double());
        std::string moveitTopic = get_parameter("moveit_cpp_sate_topic").as_string();
        debug_print_standby_state_ = get_parameter("debug.print_standby_state").as_bool();
        robot_ip_ = get_parameter("robot_ip").as_string();

        // create state publisher
        pub_motion_state_ = create_publisher<whi_interfaces::msg::WhiMotionState>("arm_motion_state", rclcpp::QoS(10));
        // subscibe the state from moveit_cpp
        if (!moveitTopic.empty())
        {
            sub_moveit_cpp_state_ = create_subscription<std_msgs::msg::Bool>(
                moveitTopic, rclcpp::QoS(10), std::bind(&UrRobotDriverBridge::callbackMoveitCppState, this, std::placeholders::_1));
        }
        // advertise io service with fixed name
        server_io_ = create_service<whi_interfaces::srv::WhiSrvIo>("io_request",
            std::bind(&UrRobotDriverBridge::onServiceIo, this, std::placeholders::_1, std::placeholders::_2));
        server_ready_ = create_service<std_srvs::srv::Trigger>("arm_ready",
            std::bind(&UrRobotDriverBridge::onServiceReady, this, std::placeholders::_1, std::placeholders::_2));
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UrRobotDriverBridge::on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Activating");

        // lifecycle publishers must be explicitly activated
        pub_motion_state_->on_activate();

        // to initiate
        int res = checkingConnection();
        if (res == MAX_TRIED)
        {
            return CallbackReturn::FAILURE;
        }
        else
        {
            if (res == REPEATED)
            {
                // buy some time for ur_robot_driver
                std::this_thread::sleep_for(std::chrono::milliseconds(ur_driver_patience_));
            }

            // spawn the safty monitor thread
            if (safty_query_duration_ > 0)
            {
                th_safty_ = std::thread(std::bind(&UrRobotDriverBridge::threadSafty, this));
            }
            beStandby();
            createBond();

            return CallbackReturn::SUCCESS;
        }
    }

    CallbackReturn UrRobotDriverBridge::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");

        // lifecycle publishers must be explicitly deactivated
        pub_motion_state_->on_deactivate();

        destroyBond();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UrRobotDriverBridge::on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        pub_motion_state_.reset();
        sub_moveit_cpp_state_.reset();
        server_io_.reset(); 
        server_ready_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UrRobotDriverBridge::on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return CallbackReturn::SUCCESS;
    }

    static bool ping(const std::string& Addr)
    {
        std::string cmd(std::string("ping ") + Addr + " -w 2");
        int res = system(cmd.c_str());

        return res == 0;
    }

    int UrRobotDriverBridge::checkingConnection()
    {
        int tryCount = 0;
        while (!ping(robot_ip_) && ++tryCount < try_max_count_)
        {
            RCLCPP_WARN_STREAM(get_logger(),
                "failed to ping:" << robot_ip_ << ", attempt to another try in " << try_duration_ / 1000.0 << " seconds");
            std::this_thread::sleep_for(std::chrono::milliseconds(try_duration_));
        }
        if (tryCount == 0)
        {
            return ONE_SHOT;
        }
        else if (tryCount >= try_max_count_)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "failed to connect UR controller");
            return MAX_TRIED;
        }
        else
        {
            return REPEATED;
        }
    }

    void UrRobotDriverBridge::beStandby()
    {
        std::thread
        {
            [this]() -> void
            {
                whi_interfaces::msg::WhiMotionState msg;
                msg.state = whi_interfaces::msg::WhiMotionState::STA_BOOTING;

                /// query the robot state till arm is started
                // service get_robot_mode
                std::string service(prefix_dashboard_ + "get_robot_mode");
                auto clientRobotMode = create_client<ur_dashboard_msgs::srv::GetRobotMode>(service);
                auto requestRobotMode = std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
                int tryCount = 0;
                std::future_status status;
                do
                {
                    auto future = clientRobotMode->async_send_request(requestRobotMode);

                    switch (status = future.wait_for(std::chrono::milliseconds(try_duration_)); status)
                    {
                    case std::future_status::deferred:
                        RCLCPP_WARN_STREAM(get_logger(), "deferred to call service " << service << ", attempt to another try in "
                            << try_duration_ << " seconds");
                        break;
                    case std::future_status::timeout:
                        RCLCPP_WARN_STREAM(get_logger(), "timeout to call service " << service << ", attempt to another try in "
                            << try_duration_ << " seconds");
                        break;
                    case std::future_status::ready:
                        RCLCPP_INFO_STREAM(get_logger(), "service call succeeded for " << service);
                        break;
                    }
                }
                while (++tryCount < try_max_count_ && status != std::future_status::ready);

                if (status != std::future_status::ready)
                {
                    RCLCPP_ERROR_STREAM(get_logger(), "failed to call service " << service);

                    return;
                }

                // check if is in remote mode
                tryCount = 0;
                while (!isInRemote())
                {
                    // publish the booting state
                    pub_motion_state_->publish(msg);

                    if (++tryCount > try_max_count_)
                    {
                        RCLCPP_ERROR_STREAM(get_logger(), "failed to boot UR since it is not in remote mode");

                        return;
                    }
                    else
                    {
                        RCLCPP_WARN_STREAM(get_logger(), "UR is not in remote mode, please set it to remote, attempt to another try in 5 seconds");
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                    }
                }
                if (tryCount > 0)
                {
                    if (disconnect() == RES_FAILED_EXECUTE || reconnect() == RES_FAILED_EXECUTE)
                    {
                        RCLCPP_ERROR_STREAM(get_logger(), "failed to booting UR due to connection issue");
                        return;
                    }
                }

                /// clear state
                // close popups
                closePopups();
                // recover from protective
                bool protective = false;
                isProtective(protective);
                if (protective)
                {
                    recoverFromProtective();
                }

                /// handle power on process
                int programRunningCount = 0;
                while (true)
                {
                    // publish the booting state
                    pub_motion_state_->publish(msg);

                    auto future = clientRobotMode->async_send_request(requestRobotMode);
                    auto result = future.get();
                    if (result->success)
                    {
                        if (result->robot_mode.mode == ur_dashboard_msgs::msg::RobotMode::POWER_OFF)
                        {
                            if (debug_print_standby_state_)
                            {
                                std::cout << "beStandby state calling ------> power on" << std::endl;
                            }
                            if (powerOn() == RES_FAILED_EXECUTE)
                            {
                                if (debug_print_standby_state_)
                                {
                                    std::cout << "beStandby state calling power on failed ------> disconnect and reconnect" << std::endl;
                                }
                                disconnect();
                                reconnect();
                                closePopups();
                            }
                        }
                        else if (result->robot_mode.mode == ur_dashboard_msgs::msg::RobotMode::IDLE)
                        {
                            if (debug_print_standby_state_)
                            {
                                std::cout << "beStandby state calling ------> get loaded program" << std::endl;
                            }
                            if (getLoadedProgram().find(external_program_) != std::string::npos)
                            {
                                if (debug_print_standby_state_)
                                {
                                    std::cout << "beStandby state calling ------>brake release" << std::endl;
                                }
                                if (releaseBrake() == RES_FAILED_EXECUTE)
                                {
                                    if (debug_print_standby_state_)
                                    {
                                        std::cout << "beStandby state calling brake release failed ------> disconnect and reconnect" << std::endl;
                                    }
                                    disconnect();
                                    reconnect();
                                    closePopups();
                                } 
                            }
                            else
                            {
                                if (debug_print_standby_state_)
                                {
                                    std::cout << "beStandby state calling ------> program load" << std::endl;
                                }
                                if (requestLoadProgram() == RES_FAILED_EXECUTE)
                                {
                                    if (debug_print_standby_state_)
                                    {
                                        std::cout << "beStandby state calling program load failed ------> disconnect and reconnect" << std::endl;
                                    }
                                    disconnect();
                                    reconnect();
                                    closePopups();
                                }
                                else
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                }
                            }
                        }
                        else if (result->robot_mode.mode == ur_dashboard_msgs::msg::RobotMode::RUNNING)
                        {
                            if (debug_print_standby_state_)
                            {
                                std::cout << "beStandby state calling ------> get loaded program" << std::endl;
                            }
                            if (getLoadedProgram().find(external_program_) == std::string::npos)
                            {
                                if (debug_print_standby_state_)
                                {
                                    std::cout << "beStandby state calling ------> deactive running program" << std::endl;
                                }
                                int res = deactiveRunningProgram();
                                if (res == RES_FAILED_EXECUTE)
                                {
                                    if (debug_print_standby_state_)
                                    {
                                        std::cout << "beStandby state calling deactive running program failed ------> disconnect and reconnect" << std::endl;
                                    }
                                    disconnect();
                                    reconnect();
                                    closePopups();
                                }
                                else
                                {
                                    if (debug_print_standby_state_)
                                    {
                                        std::cout << "beStandby state calling ------> program load" << std::endl;
                                    }
                                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                    if (requestLoadProgram() == RES_FAILED_EXECUTE)
                                    {
                                        if (debug_print_standby_state_)
                                        {
                                            std::cout << "beStandby state calling program load failed ------> disconnect and reconnect" << std::endl;
                                        }
                                        disconnect();
                                        reconnect();
                                        closePopups();
                                    }
                                    else
                                    {
                                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                        if (powerOff() == RES_FAILED_EXECUTE)
                                        {
                                            disconnect();
                                            reconnect();
                                            closePopups();
                                        }
                                    }
                                }
                            }
                            else
                            {
                                if (debug_print_standby_state_)
                                {
                                    std::cout << "beStandby state calling ------> is program running" << std::endl;
                                }
                                if (!isProgramRunning())
                                {
                                    if (debug_print_standby_state_)
                                    {
                                        std::cout << "beStandby state calling ------> request play" << std::endl;
                                    }
                                    if (requestPlay() == RES_FAILED_EXECUTE)
                                    {
                                        if (debug_print_standby_state_)
                                        {
                                            std::cout << "beStandby state calling request play failed ------> disconnect and reconnect" << std::endl;
                                        }
                                        disconnect();
                                        reconnect();
                                        closePopups();
                                    }
                                }
                                else
                                {
                                    if (debug_print_standby_state_)
                                    {
                                        std::cout << "beStandby state ------> robot and program is running" << std::endl;
                                    }
                                    if (++programRunningCount >= 3)
                                    {
                                        std::lock_guard<std::mutex> lock(mtx_);
                                        standby_ = true;
                                        cv_.notify_all();

                                        break;
                                    }
                                    else
                                    {
                                        closePopups();
                                    }
                                }
                            }
                        }                  
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(get_logger(), "failed to call service " << service);
                    }

                    if (debug_print_standby_state_)
                    {
                        std::cout << "beStandby state robot mode in loop " << int(result->robot_mode.mode) << std::endl;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
        }.detach();
    }

    void UrRobotDriverBridge::threadSafty()
    {
        {
		    std::unique_lock lock(mtx_);
    	    cv_.wait(lock, [this]{ return standby_; });
	    }

        // set payload
        if (!setPayload())
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to set payload");
        }

        bool disconnected = false;
        while (!terminated_.load())
	    {
            if (disconnected)
            {
                reconnect();
                disconnected = closePopups() != RES_FAILED_EXECUTE ? false : true;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            else
            {
                /// query the safty state
                bool protective = false;
                if (isProtective(protective, (terminated_.load() ? false : true)) == RES_SUCCEED)
                {
                    if (protective)
                    {
                        // notify depended ones
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        standby_ = false;

                        // recover
                        int res = recoverFromProtective();
                        if (res != RES_FAILED_EXECUTE)
                        {
                            // close popups
                            closePopups();

                            // stop program otherwise "other thread is running" will be poped out
                            stopProgram();

                            // reload and replay
                            if (requestPlay() != RES_FAILED_EXECUTE)
                            {
                                standby_ = true;
                            }
                            else
                            {
                                disconnected = true;
                            }
                        }
                        else
                        {
                            disconnected = true;
                        }
                    }
                    else
                    {
                        // send standby message
                        whi_interfaces::msg::WhiMotionState msg;
                        if (sub_moveit_cpp_state_)
                        {
                            msg.state = moveit_cpp_ready_ ?
                                whi_interfaces::msg::WhiMotionState::STA_STANDBY : whi_interfaces::msg::WhiMotionState::STA_BOOTING;
                        }
                        else
                        {
                            msg.state = whi_interfaces::msg::WhiMotionState::STA_STANDBY;
                        }
                        pub_motion_state_->publish(msg);
                    }
                }
                else
                {
                    disconnected = true;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(safty_query_duration_));
        }
    }

    std::string UrRobotDriverBridge::getLoadedProgram()
    {
        std::string loaded;

        std::string service(prefix_dashboard_ + "get_loaded_program");
        auto clientLoaded = create_client<ur_dashboard_msgs::srv::GetLoadedProgram>(service);
        auto request = std::make_shared<ur_dashboard_msgs::srv::GetLoadedProgram::Request>();
        auto future = clientLoaded->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            loaded.assign(result->program_name);
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "failed to call service " << service);
        }

        return loaded;
    }

    int UrRobotDriverBridge::requestLoadProgram()
    {
        // service load_program
        std::string service(prefix_dashboard_ + "load_program");
        auto clientLoadProgram = create_client<ur_dashboard_msgs::srv::Load>(service);
        auto request = std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
        request->filename = external_program_;
        auto future = clientLoadProgram->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "program " << request->filename << " is loaded successfully");
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service << " to load program file "
                << request->filename);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::deactiveRunningProgram()
    {
        std::string service(prefix_dashboard_ + "program_running");
        auto clientRunning = create_client<ur_dashboard_msgs::srv::IsProgramRunning>(service);
        auto request = std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>();
        auto future = clientRunning->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            if (result->program_running)
            {
                return stopProgram();
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "program is not running yet");
                return RES_SUCCEED;
            }
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::requestPlay()
    {
        // service play
        std::string service(prefix_dashboard_ + "play");
        auto clientPlay = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientPlay->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "UR is standby");
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::powerOn()
    {
        // service power_on
        std::string service(prefix_dashboard_ + "power_on");
        auto clientPowerOn = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientPowerOn->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "power on successfully");
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::powerOff(bool Sync/* = true*/)
    {
        // service power_off
        std::string service(prefix_dashboard_ + "power_off");
        auto clientPowerOff = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientPowerOff->async_send_request(request);
        if (Sync)
        {
            auto result = future.get();
            if (result->success)
            {
                RCLCPP_INFO_STREAM(get_logger(), "power off successfully");
                return RES_SUCCEED;
            }
            else
            {
                RCLCPP_ERROR_STREAM(get_logger(), "failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            return RES_SUCCEED;
        }
    }

    int UrRobotDriverBridge::releaseBrake()
    {
        // service brake_release
        std::string service(prefix_dashboard_ + "brake_release");
        auto clientBrakeRelease = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientBrakeRelease->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "brake released successfully");
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::closePopups()
    {
        // non-safty
        std::string service(prefix_dashboard_ + "close_popup");
        auto clientNonSafty = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientNonSafty->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "non-safety popup is closed successfully");
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }

        // safty
        service.assign(prefix_dashboard_ + "close_safety_popup");
        auto clientSafty = create_client<std_srvs::srv::Trigger>(service);
        future = clientSafty->async_send_request(request);
        result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "safety popup is closed successfully");
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::isProtective(bool& IsProtective, bool Sync/* = true*/)
    {
        // service get_safty_mode
        std::string service(prefix_dashboard_ + "get_safety_mode");
        auto clientSafetyMode = create_client<ur_dashboard_msgs::srv::GetSafetyMode>(service);
        auto request = std::make_shared<ur_dashboard_msgs::srv::GetSafetyMode::Request>();
        auto future = clientSafetyMode->async_send_request(request);
        if (Sync)
        {
            auto result = future.get();
            if (result->success)
            {
                if (result->safety_mode.mode == ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP)
                {
                    // let the "External Control" program node on the UR-Program return
                    handBackControl();

                    // cancel all goals preemption policy
                    auto actionClient = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                        this, "scaled_joint_trajectory_controller/follow_joint_trajectory");
                    actionClient->async_cancel_all_goals();

                    whi_interfaces::msg::WhiMotionState msg;
                    msg.state = whi_interfaces::msg::WhiMotionState::STA_FAULT;
                    pub_motion_state_->publish(msg);

                    IsProtective = true;

                    RCLCPP_WARN_STREAM(get_logger(), "UR entered protective stop state");
                }
                else
                {
                    IsProtective = false;
                }

                return RES_SUCCEED;
            }
            else
            {
                IsProtective = false;

                RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            IsProtective = false;
            return RES_SUCCEED;
        }
    }

    int UrRobotDriverBridge::recoverFromProtective()
    {
        // unlock protective
        std::string service(prefix_dashboard_ + "unlock_protective_stop");
        auto clientUnlockProtective = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientUnlockProtective->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "UR is recovered from protective state");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return RES_FAILED_EXECUTE;
        }
    }

    int UrRobotDriverBridge::stopProgram(bool Sync/* = true*/)
    {
        std::string service(prefix_dashboard_ + "stop");
        auto clientStop = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientStop->async_send_request(request);
        if (Sync)
        {
            auto result = future.get();
            if (result->success)
            {
                RCLCPP_INFO_STREAM(get_logger(), "stop running program successfully");
                return RES_SUCCEED;
            }
            else
            {
                RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service << " to deactivate program");
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            return RES_SUCCEED;
        }
    }

    bool UrRobotDriverBridge::isInRemote()
    {
        // service is_in_remote_control
        std::string service(prefix_dashboard_ + "is_in_remote_control");
        auto clientRemote = create_client<ur_dashboard_msgs::srv::IsInRemoteControl>(service);
        auto request = std::make_shared<ur_dashboard_msgs::srv::IsInRemoteControl::Request>();
        auto future = clientRemote->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            return result->remote_control;
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "failed to call service " << service);
            return false;
        }
    }

    bool UrRobotDriverBridge::isProgramRunning()
    {
        // service program_state
        std::string service(prefix_dashboard_ + "program_state");
        auto client = create_client<ur_dashboard_msgs::srv::GetProgramState>(service);
        auto request = std::make_shared<ur_dashboard_msgs::srv::GetProgramState::Request>();
        auto future = client->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            return result->state.state == ur_dashboard_msgs::msg::ProgramState::PLAYING;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to execute service " << service);
            return false;
        } 
    }

    int UrRobotDriverBridge::disconnect(bool Sync/* = true*/)
    {
        std::string service(prefix_dashboard_ + "quit");
        auto clientQuit = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientQuit->async_send_request(request);
        if (Sync)
        {
            auto result = future.get();
            if (result->success)
            {
                RCLCPP_INFO_STREAM(get_logger(), "disconnect successfully");
                return RES_SUCCEED;
            }
            else
            {
                RCLCPP_WARN_STREAM(get_logger(), "failed to disconnect");
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            return RES_SUCCEED;
        }
    }

    int UrRobotDriverBridge::reconnect()
    {
        std::string service(prefix_dashboard_ + "connect");
        auto clientConnect = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientConnect->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO_STREAM(get_logger(), "reconnect successfully");
            return RES_SUCCEED;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "failed to reconnect");
            return RES_FAILED_EXECUTE;
        }
    }

    bool UrRobotDriverBridge::setPayload()
    {
        std::string service("io_and_status_controller/set_payload");
        auto client = create_client<ur_msgs::srv::SetPayload>(service);
        auto request = std::make_shared<ur_msgs::srv::SetPayload::Request>();
        request->mass = payload_weight_;
        request->center_of_gravity.x = payload_to_tcp_[0];
        request->center_of_gravity.y = payload_to_tcp_[1];
        request->center_of_gravity.z = payload_to_tcp_[2];
        auto future = client->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            return true;
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "failed to execute service " << service);
            return false;
        }
    }

    bool UrRobotDriverBridge::handBackControl()
    {
        std::string service("io_and_status_controller/hand_back_control");
        auto clientHandback = create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = clientHandback->async_send_request(request);
        auto result = future.get();
        if (result->success)
        {
            return true;
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "failed to call service " << service);
            return false;
        }
    }

    void UrRobotDriverBridge::onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> Response)
    {
        // service SetIO
        if (Request->io.operation == whi_interfaces::msg::WhiIo::OPER_WRITE)
        {
            std::string service("io_and_status_controller/set_io");
            auto client = create_client<ur_msgs::srv::SetIO>(service);
            auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
            request->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
            request->pin = Request->io.addr;
            request->state = Request->io.level;
            auto future = client->async_send_request(request);
            auto result = future.get();
            if (result->success)
            {
                Response->result = true;
            }
            else
            {
                Response->result = false;
                RCLCPP_ERROR_STREAM(get_logger(), "failed to execute service " << service);
            }
        }
        else
        {
            Response->result = false;
            RCLCPP_ERROR_STREAM(get_logger(), "only write operation is supported in UR");
        }
    }

    void UrRobotDriverBridge::onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> Response)
    {
        Response->success = standby_;
    }

    void UrRobotDriverBridge::callbackMoveitCppState(const std_msgs::msg::Bool::SharedPtr Msg)
    {
        moveit_cpp_ready_ = Msg->data;
    }
} // namespace whi_ur_robot_driver_bridge
