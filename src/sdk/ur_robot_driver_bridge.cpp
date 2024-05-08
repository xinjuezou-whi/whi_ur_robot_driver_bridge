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
#include "whi_interfaces/WhiMotionState.h"

#include <ur_dashboard_msgs/GetRobotMode.h>
#include <ur_dashboard_msgs/GetProgramState.h>
#include <ur_dashboard_msgs/GetSafetyMode.h>
#include <ur_dashboard_msgs/GetLoadedProgram.h>
#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/IsProgramRunning.h>
#include <ur_msgs/SetIO.h>
#include <std_srvs/Trigger.h>

#include <thread>

namespace whi_ur_robot_driver_bridge
{
    UrRobotDriverBridge::UrRobotDriverBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle), node_handle_ns_free_(std::make_shared<ros::NodeHandle>())
    {
        init();
    }

    UrRobotDriverBridge::~UrRobotDriverBridge()
    {
        if (client_power_off_)
        {
            std_srvs::Trigger srv;
            client_power_off_->call(srv);
        }
        disconnect();

        terminated_.store(true);
	    if (th_safty_.joinable())
	    {
		    th_safty_.join();
	    }
    }

    void UrRobotDriverBridge::init()
    {
        // params
        std::string ns = ros::this_node::getNamespace();
        if (!ns.empty())
        {
            service_prefix_ = ns + "/" + service_prefix_;
        }
        node_handle_->param("try_duration", try_duration_, 2);
        node_handle_->param("try_max_count", try_max_count_, 10);
        node_handle_->param("external_program", external_program_, std::string("external_ctrl.urp"));
        double frequency = 0.0;
        node_handle_->param("protective_query_frequency", frequency, 0.0);
        if (frequency > 1e-5)
        {
            safty_query_duration_ = int(1000.0 / frequency);
        }

        beStandby();

        if (safty_query_duration_ > 0)
        {
            // create state publisher
            std::string stateTopic;
            node_handle_->param("state_topic", stateTopic, std::string("arm_moton_state"));
            pub_motion_state_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<whi_interfaces::WhiMotionState>(stateTopic, 1));
            // spawn the safty monitor thread
		    th_safty_ = std::thread(std::bind(&UrRobotDriverBridge::threadSafty, this));
        }

        // advertise io service with fixed name
		server_io_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("io_request", &UrRobotDriverBridge::onServiceIo, this));
        server_ready_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("arm_ready", &UrRobotDriverBridge::onServiceReady, this));
    }

    void UrRobotDriverBridge::beStandby()
    {
        std::thread
        {
            [this]() -> void
            {
                /// query the robot state till arm is started
                // service get_robot_mode
                std::string service(service_prefix_ + prefix_dashboard_ + "get_robot_mode");
                auto clientRobotMode = std::make_unique<ros::ServiceClient>(
                    node_handle_ns_free_->serviceClient<ur_dashboard_msgs::GetRobotMode>(service));
                ur_dashboard_msgs::GetRobotMode srvRobotMode;
                int tryCount = 0;
                while (!clientRobotMode->call(srvRobotMode))
                {
                    if (++tryCount > this->try_max_count_)
                    {
                        ROS_ERROR_STREAM("failed to call service " << service);

                        return;
                    }
                    else
                    {
                        ROS_WARN_STREAM("failed to call service, attempt to another try in "
                            << this->try_duration_ << " seconds");
                        std::this_thread::sleep_for(std::chrono::seconds(this->try_duration_));
                    }
                }

                /// clear state
                // close popups
                closePopups();
                // recover from protective
                bool proceeding = true;
                if (isProtective())
                {
                    proceeding = recoverFromProtective();
                }

                /// handle power on process
                if (proceeding)
                {
                    while (!client_power_off_ && clientRobotMode->call(srvRobotMode))
                    {
                        switch (srvRobotMode.response.robot_mode.mode)
                        {
                        case ur_dashboard_msgs::RobotMode::POWER_OFF:
                            {
                                // service power_on
                                service.assign(service_prefix_ + prefix_dashboard_ + "power_on");
                                auto clientPowerOn = std::make_unique<ros::ServiceClient>(
                                    node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
                                std_srvs::Trigger srv;
                                if (!clientPowerOn->call(srv))
                                {
                                    proceeding = false;
                                    ROS_ERROR_STREAM("failed to call service " << service);
                                }
                            }
                            break;
                        case ur_dashboard_msgs::RobotMode::IDLE:
                            {
                                // service brake_release
                                static bool responseSucceed = false;
                                static int tryCount = 0;
                                if (!responseSucceed)
                                {
                                    service.assign(service_prefix_ + prefix_dashboard_ + "brake_release");
                                    auto clientBrakeRelease = std::make_unique<ros::ServiceClient>(
                                        node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
                                    std_srvs::Trigger srv;
                                    if (!clientBrakeRelease->call(srv))
                                    {
                                        proceeding = false;
                                        ROS_ERROR_STREAM("failed to call service " << service);
                                    }
                                    else
                                    {
                                        responseSucceed = srv.response.success;
                                        if (!responseSucceed && ++tryCount > 5)
                                        {
                                            proceeding = false;
                                            ROS_ERROR_STREAM("failed to execute service " << service);
                                        }
                                    }
                                }
                            }
                            break;
                        case ur_dashboard_msgs::RobotMode::RUNNING:
                            // service power_off
                            service.assign(service_prefix_ + prefix_dashboard_ + "power_off");
                            client_power_off_ = std::make_unique<ros::ServiceClient>(
                                node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
                            break;
                        default:
                            break;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                }

                if (proceeding && requestLoadProgram())
                {
                    requestPlay();
                    // connection will be refused at startup while there is an autorunning program
                    // trick is to disconnect and reconnect
                    disconnect();
                    reconnect();
                    closePopups();

                    std::lock_guard<std::mutex> lock(mtx_);
                    standby_ = true;
                    cv_.notify_all();
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

        while (!terminated_.load())
	    {
            /// query the safty state
            if (isProtective())
            {
                standby_ = false;

                // recover and then reload the program
                if (recoverFromProtective())
                {
                    // close popups
                    closePopups();
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));

                    // reload and replay
                    if (requestLoadProgram() && requestPlay())
                    {
                        standby_ = true;
                    }

                    // send recovered message
                    whi_interfaces::WhiMotionState msg;
                    msg.state = whi_interfaces::WhiMotionState::STA_STANDBY;
                    pub_motion_state_->publish(msg);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(safty_query_duration_));
        }
    }

    bool UrRobotDriverBridge::requestLoadProgram()
    {
        bool res = false;

        // deactive running program first
        std::string service(service_prefix_ + prefix_dashboard_ + "program_running");
        auto clientRunning = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::IsProgramRunning>(service));
        ur_dashboard_msgs::IsProgramRunning srvRunning;
        if (clientRunning->call(srvRunning))
        {
            if (srvRunning.response.program_running)
            {
                service.assign(service_prefix_ + prefix_dashboard_ + "stop");
                auto clientStop = std::make_unique<ros::ServiceClient>(
                    node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
                std_srvs::Trigger srv;
                if (clientStop->call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO_STREAM("stop running program successfully");
                    }
                }
            }
        }

        // service load_program
        service.assign(service_prefix_ + prefix_dashboard_ + "load_program");
        auto clientLoadProgram = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::Load>(service));
        ur_dashboard_msgs::Load srvLoadProgram;
        srvLoadProgram.request.filename = external_program_;
        if (clientLoadProgram->call(srvLoadProgram))
        {
            if (srvLoadProgram.response.success)
            {
                res = true;
                ROS_INFO_STREAM("program " << srvLoadProgram.request.filename << " is loaded successfully");
            }
            else
            {
                ROS_ERROR_STREAM("failed to execute service " << service << " to load program file "
                    << srvLoadProgram.request.filename);
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return res;
    }

    bool UrRobotDriverBridge::requestPlay()
    {
        bool res = false;

        // service play
        std::string service(service_prefix_ + prefix_dashboard_ + "play");
        auto clientPlay = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientPlay->call(srv))
        {
            if (srv.response.success)
            {
                res = true;
                ROS_INFO_STREAM("UR is standby");
            }
            else
            {
                ROS_ERROR_STREAM("failed to execute service " << service);
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return res;
    }

    bool UrRobotDriverBridge::closePopups()
    {
        bool res = true;

        // non-safty
        std::string service(service_prefix_ + prefix_dashboard_ + "close_popup");
        auto clientNonSafty = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientNonSafty->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("non safety popup is closed successfully");
            }
            else
            {
                res &= false;
                ROS_ERROR_STREAM("failed to execute service " << service);
            }
        }
        else
        {
            res &= false;
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        // safty
        service.assign(service_prefix_ + prefix_dashboard_ + "close_safety_popup");
        auto clientSafty = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        if (clientSafty->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("safety popup is closed successfully");
            }
            else
            {
                res &= false;
                ROS_ERROR_STREAM("failed to execute service " << service);
            }
        }
        else
        {
            res &= false;
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return res;
    }

    bool UrRobotDriverBridge::isProtective()
    {
        // service get_safty_mode
        std::string service(service_prefix_ + prefix_dashboard_ + "get_safety_mode");
        auto clientSafetyMode = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::GetSafetyMode>(service));
        ur_dashboard_msgs::GetSafetyMode srvSafetyMode;
        clientSafetyMode->call(srvSafetyMode);
        if (srvSafetyMode.response.safety_mode.mode == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP)
        {
            whi_interfaces::WhiMotionState msg;
            msg.state = whi_interfaces::WhiMotionState::STA_FAULT;
            pub_motion_state_->publish(msg);

            ROS_WARN_STREAM("UR entered protective stop state");
            return true;
        }

        return false;
    }

    bool UrRobotDriverBridge::recoverFromProtective()
    {
        bool res = false;

        // unlock protective
        std::string service(service_prefix_ + prefix_dashboard_ + "unlock_protective_stop");
        auto clientUnlockProtective = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (!clientUnlockProtective->call(srv))
        {
            ROS_ERROR_STREAM("failed to call service " << service);
        }
        else
        {
            if (srv.response.success)
            {
                res = true;
                ROS_INFO_STREAM("UR is recovered from protective state");
            }
            else
            {
                ROS_ERROR_STREAM("failed to execute service " << service);
            }
        }

        if (res)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        
        return res;
    }

    bool UrRobotDriverBridge::disconnect()
    {
        bool res = false;

        std::string service(service_prefix_ + prefix_dashboard_ + "quit");
        auto clientQuit = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientQuit->call(srv))
        {
            if (srv.response.success)
            {
                res = true;
                ROS_INFO_STREAM("disconnect successfully");
            }
            else
            {
                ROS_ERROR_STREAM("failed to disconnect");
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return res;
    }

    bool UrRobotDriverBridge::reconnect()
    {
        bool res = false;

        std::string service(service_prefix_ + prefix_dashboard_ + "connect");
        auto clientConnect = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientConnect->call(srv))
        {
            if (srv.response.success)
            {
                res = true;
                ROS_INFO_STREAM("reconnect successfully");
            }
            else
            {
                ROS_ERROR_STREAM("failed to reconnect");
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return res;
    }

    bool UrRobotDriverBridge::onServiceIo(whi_interfaces::WhiSrvIo::Request& Request,
        whi_interfaces::WhiSrvIo::Response& Response)
    {
        // service SetIO
        if (Request.operation == whi_interfaces::WhiSrvIo::Request::OPER_WRITE)
        {
            std::string service(service_prefix_ + "set_io");
            auto client = std::make_unique<ros::ServiceClient>(
                node_handle_->serviceClient<ur_msgs::SetIO>(service));
            ur_msgs::SetIO srv;
            srv.request.fun = ur_msgs::SetIO::Request::FUN_SET_DIGITAL_OUT;
            srv.request.pin = Request.addr;
            srv.request.state = Request.level;
            if (client->call(srv))
            {
                Response.result = srv.response.success;
                if (!Response.result)
                {
                    ROS_ERROR_STREAM("failed to execute service " << service);
                }
            }
            else
            {
                ROS_ERROR_STREAM("failed to call service " << service);
            }            
        }
        else
        {
            Response.result = false;
            ROS_ERROR_STREAM("only write operation is supported in UR");
        }

        return Response.result;
    }

    bool UrRobotDriverBridge::onServiceReady(std_srvs::Trigger::Request& Request, std_srvs::Trigger::Response& Response)
    {
        return (Response.success = standby_);
    }
} // namespace whi_ur_robot_driver_bridge
