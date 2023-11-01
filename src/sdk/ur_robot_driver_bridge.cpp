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
#include <ur_dashboard_msgs/GetRobotMode.h>
#include <ur_dashboard_msgs/GetProgramState.h>
#include <ur_dashboard_msgs/GetSafetyMode.h>
#include <ur_dashboard_msgs/Load.h>
#include <std_srvs/Trigger.h>

#include <thread>

namespace whi_ur_robot_driver_bridge
{
    UrRobotDriverBridge::UrRobotDriverBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
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

        terminated_.store(true);
	    if (th_safty_.joinable())
	    {
		    th_safty_.join();
	    }
    }

    void UrRobotDriverBridge::init()
    {
        // params
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
            // spawn the safty monitor thread
		    th_safty_ = std::thread(std::bind(&UrRobotDriverBridge::threadSafty, this));
        }
    }

    void UrRobotDriverBridge::beStandby()
    {
        std::thread
        {
            [this]() -> void
            {
                bool proceed = true;

                /// query the robot state
                // service get_robot_mode
                std::string service("/ur_hardware_interface/dashboard/get_robot_mode");
                auto clientRobotMode = std::make_unique<ros::ServiceClient>(
                    node_handle_->serviceClient<ur_dashboard_msgs::GetRobotMode>(service));
                ur_dashboard_msgs::GetRobotMode srvRobotMode;
                int tryCount = 0;
                while (!clientRobotMode->call(srvRobotMode))
                {
                    if (++tryCount > this->try_max_count_)
                    {
                        proceed = false;
                        ROS_ERROR_STREAM("failed to call service " << service);
                        break;
                    }
                    else
                    {
                        ROS_WARN_STREAM("failed to call service, attempt to another try in "
                            << this->try_duration_ << " seconds");
                        std::this_thread::sleep_for(std::chrono::seconds(this->try_duration_));
                    }
                }

                /// handle power on process
                while (proceed && clientRobotMode->call(srvRobotMode))
                {
                    switch (srvRobotMode.response.robot_mode.mode)
                    {
                    case ur_dashboard_msgs::RobotMode::POWER_OFF:
                        {
                            // service power_on
                            service = "/ur_hardware_interface/dashboard/power_on";
                            auto clientPowerOn = std::make_unique<ros::ServiceClient>(
                                node_handle_->serviceClient<std_srvs::Trigger>(service));
                            std_srvs::Trigger srv;
                            if (!clientPowerOn->call(srv))
                            {
                                proceed = false;
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
                                service = "/ur_hardware_interface/dashboard/brake_release";
                                auto clientBrakeRelease = std::make_unique<ros::ServiceClient>(
                                    node_handle_->serviceClient<std_srvs::Trigger>(service));
                                std_srvs::Trigger srv;
                                if (!clientBrakeRelease->call(srv))
                                {
                                    proceed = false;
                                    ROS_ERROR_STREAM("failed to call service " << service);
                                }
                                else
                                {
                                    responseSucceed = srv.response.success;
                                    if (!responseSucceed && ++tryCount > 5)
                                    {
                                        proceed = false;
                                        ROS_ERROR_STREAM("failed to execute service " << service);
                                    }
                                }
                            }
                        }
                        break;
                    case ur_dashboard_msgs::RobotMode::RUNNING:
                        // service power_off
                        service = "/ur_hardware_interface/dashboard/power_off";
                        client_power_off_ = std::make_unique<ros::ServiceClient>(
                            node_handle_->serviceClient<std_srvs::Trigger>(service));
                        proceed = false;
                        break;
                    default:
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }

                /// load urp program and play
                if (requestLoadProgram())
                {
                    if (requestPlay())
                    {
                        std::lock_guard<std::mutex> lock(mtx_);
			            standby_ = true;
                        cv_.notify_all();
                    }
                }
            }
        }.detach();
    }

    bool UrRobotDriverBridge::requestLoadProgram()
    {
        bool res = false;

        // service load_program
        std::string service("/ur_hardware_interface/dashboard/load_program");
        auto clientLoadProgram = std::make_unique<ros::ServiceClient>(
            node_handle_->serviceClient<ur_dashboard_msgs::Load>(service));
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
        std::string service("/ur_hardware_interface/dashboard/play");
        auto clientPlay = std::make_unique<ros::ServiceClient>(
            node_handle_->serviceClient<std_srvs::Trigger>(service));
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

    void UrRobotDriverBridge::threadSafty()
    {
        {
		    std::unique_lock lock(mtx_);
    	    cv_.wait(lock, [this]{ return standby_; });
	    }

        while (!terminated_.load())
	    {
            /// query the safty state
            // service get_safty_mode
            std::string service("/ur_hardware_interface/dashboard/get_safety_mode");
            auto clientSafetyMode = std::make_unique<ros::ServiceClient>(
                node_handle_->serviceClient<ur_dashboard_msgs::GetSafetyMode>(service));
            ur_dashboard_msgs::GetSafetyMode srvSafetyMode;
            clientSafetyMode->call(srvSafetyMode);
            if (srvSafetyMode.response.safety_mode.mode == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP)
            {
                ROS_WARN_STREAM("UR entered protective stop state");

                // unlock protective
                service = "/ur_hardware_interface/dashboard/unlock_protective_stop";
                auto clientUnlockProtective = std::make_unique<ros::ServiceClient>(
                    node_handle_->serviceClient<std_srvs::Trigger>(service));
                std_srvs::Trigger srv;
                if (!clientUnlockProtective->call(srv))
                {
                    ROS_ERROR_STREAM("failed to call service " << service);
                }
                else
                {
                    if (srv.response.success)
                    {
                        ROS_INFO_STREAM("UR is recovered from protective state");
                    }
                    else
                    {
                        ROS_ERROR_STREAM("failed to execute service " << service);
                    }
                }
                // reload program
                if (requestLoadProgram())
                {
                    // replay
                    requestPlay();
                    if (safty_query_duration_ < 300)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(safty_query_duration_));
        }
    }
} // namespace whi_ur_robot_driver_bridge
