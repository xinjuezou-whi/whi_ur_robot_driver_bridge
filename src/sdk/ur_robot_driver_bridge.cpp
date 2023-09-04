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
#include <ur_dashboard_msgs/GetRobotMode.h>
#include <ur_dashboard_msgs/GetProgramState.h>
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
    }

    void UrRobotDriverBridge::init()
    {
        // params
        node_handle_->param("try_duration", try_duration_, 2);
        node_handle_->param("try_max_count", try_max_count_, 10);
        node_handle_->param("external_program", external_program_, std::string("external_ctrl.urp"));

        beStandby();
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

                /// load urp program
                proceed = true;
                // service program_state
                service = "/ur_hardware_interface/dashboard/program_state";
                auto clientProgramState = std::make_unique<ros::ServiceClient>(
                    node_handle_->serviceClient<ur_dashboard_msgs::GetProgramState>(service));
                ur_dashboard_msgs::GetProgramState srvProgramState;
                while (proceed && clientProgramState->call(srvProgramState))
                {
                    if (srvProgramState.response.program_name.find("urp") == std::string::npos ||
                        srvProgramState.response.program_name != this->external_program_)
                    {
                        // service load_program
                        service = "/ur_hardware_interface/dashboard/load_program";
                        auto clientLoadProgram = std::make_unique<ros::ServiceClient>(
                            node_handle_->serviceClient<ur_dashboard_msgs::Load>(service));
                        ur_dashboard_msgs::Load srvLoadProgram;
                        srvLoadProgram.request.filename = this->external_program_;
                        if (!clientLoadProgram->call(srvLoadProgram))
                        {
                            proceed = false;
                            ROS_ERROR_STREAM("failed to call service " << service);
                        }
                        else
                        {
                            if (!srvLoadProgram.response.success)
                            {
                                proceed = false;
                                ROS_ERROR_STREAM("failed to execute service " << service << " with filename "
                                    << srvLoadProgram.request.filename);
                            }
                        }
                    }
                    else
                    {
                        proceed = false;

                        // service play
                        service = "/ur_hardware_interface/dashboard/play";
                        auto clientPlay = std::make_unique<ros::ServiceClient>(
                            node_handle_->serviceClient<std_srvs::Trigger>(service));
                        std_srvs::Trigger srv;
                        if (!clientPlay->call(srv))
                        {
                            ROS_ERROR_STREAM("failed to call service " << service);
                        }
                        else
                        {
                            if (srv.response.success)
                            {
                                ROS_INFO_STREAM("UR is standby");
                            }
                            else
                            {
                                ROS_ERROR_STREAM("failed to execute service " << service);
                            }
                        }
                    }
                }
            }
        }.detach();
    }
} // namespace whi_ur_robot_driver_bridge
