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
#include <ur_dashboard_msgs/IsInRemoteControl.h>
#include <ur_msgs/SetPayload.h>
#include <ur_msgs/SetIO.h>
#include <std_srvs/Trigger.h>

#include <thread>

// #define DEBUG

namespace whi_ur_robot_driver_bridge
{
    UrRobotDriverBridge::UrRobotDriverBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle), node_handle_ns_free_(std::make_shared<ros::NodeHandle>())
    {
        init();
    }

    UrRobotDriverBridge::~UrRobotDriverBridge()
    {
        powerOff();
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
        node_handle_->param("try_duration", try_duration_, 10);
        node_handle_->param("try_max_count", try_max_count_, 20);
        node_handle_->param("external_program", external_program_, std::string("external_ctrl.urp"));

        // create state publisher
        pub_motion_state_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiMotionState>("arm_motion_state", 1));

        // safty monitoring thread
        double frequency = 0.0;
        node_handle_->param("protective_query_frequency", frequency, 0.0);
        if (frequency > 1e-5)
        {
            safty_query_duration_ = int(1000.0 / frequency);
        }
        if (safty_query_duration_ > 0)
        {
            // spawn the safty monitor thread
		    th_safty_ = std::thread(std::bind(&UrRobotDriverBridge::threadSafty, this));
        }

        // to initiate
        beStandby();

        // advertise io service with fixed name
		server_io_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("io_request", &UrRobotDriverBridge::onServiceIo, this));
        server_ready_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("arm_ready", &UrRobotDriverBridge::onServiceReady, this));

        // subscibe the state from moveit_cpp
        std::string moveitTopic;
        node_handle_->param("moveit_cpp_state_topic", moveitTopic, std::string(""));
        if (!moveitTopic.empty())
        {
            sub_moveit_cpp_state_ = std::make_unique<ros::Subscriber>(
			    node_handle_ns_free_->subscribe<std_msgs::Bool>(moveitTopic, 10,
			    std::bind(&UrRobotDriverBridge::callbackMotionState, this, std::placeholders::_1)));
        }
    }

    void UrRobotDriverBridge::beStandby()
    {
        std::thread
        {
            [this]() -> void
            {
                whi_interfaces::WhiMotionState msg;
                msg.state = whi_interfaces::WhiMotionState::STA_BOOTING;

                /// query the robot state till arm is started
                // service get_robot_mode
                std::string service(service_prefix_ + prefix_dashboard_ + "get_robot_mode");
                auto clientRobotMode = std::make_unique<ros::ServiceClient>(
                    node_handle_ns_free_->serviceClient<ur_dashboard_msgs::GetRobotMode>(service));
                ur_dashboard_msgs::GetRobotMode srvRobotMode;
                int tryCount = 0;
                while (!clientRobotMode->call(srvRobotMode))
                {
                    // publish the booting state
                    pub_motion_state_->publish(msg);

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
                // check if is in remote mode
                tryCount = 0;
                while (!isInRemote())
                {
                    // publish the booting state
                    pub_motion_state_->publish(msg);

                    if (++tryCount > this->try_max_count_)
                    {
                        ROS_ERROR_STREAM("failed to boot UR since it is not in remote mode");

                        return;
                    }
                    else
                    {
                        ROS_WARN_STREAM("UR is not in remote mode, please set it to remote, attempt to another try in 5 seconds");
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                    }
                }
                if (tryCount > 0)
                {
                    if (disconnect() == RES_FAILED_CALL || reconnect() == RES_FAILED_CALL)
                    {
                        ROS_ERROR_STREAM("failed to booting UR due to connection issue");
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
                while (clientRobotMode->call(srvRobotMode))
                {
                    // publish the booting state
                    pub_motion_state_->publish(msg);

#ifdef DEBUG
                    std::cout << "mode in looppppppppppp " << int(srvRobotMode.response.robot_mode.mode) << std::endl;
#endif        
                    if (srvRobotMode.response.robot_mode.mode == ur_dashboard_msgs::RobotMode::POWER_OFF)
                    {
#ifdef DEBUG
                        std::cout << "state power offffffffffffffff calling power on" << std::endl;
#endif       
                        if (powerOn() == RES_FAILED_CALL)
                        {
#ifdef DEBUG
                            std::cout << "state power offffffffffffffff calling disconnect and reconnect" << std::endl;
#endif  
                            disconnect();
                            reconnect();
                            closePopups();
                        }
                    }
                    else if (srvRobotMode.response.robot_mode.mode == ur_dashboard_msgs::RobotMode::IDLE)
                    {
                        if (getLoadedProgram().find(external_program_) != std::string::npos)
                        {
#ifdef DEBUG
                            std::cout << "state idleeeeeeeeeeeeeeee calling brake release" << std::endl;
#endif  
                            if (releaseBrake() == RES_FAILED_CALL)
                            {
                                disconnect();
                                reconnect();
                                closePopups();
                            } 
                        }
                        else
                        {
#ifdef DEBUG
    std::cout << "state idleeeeeeeeeeeeeeee calling program load" << std::endl;
#endif  
                            if (requestLoadProgram() == RES_FAILED_CALL)
                            {
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
                    else if (srvRobotMode.response.robot_mode.mode == ur_dashboard_msgs::RobotMode::RUNNING)
                    {
#ifdef DEBUG
    std::cout << "state runninggggggggggggg" << std::endl;
#endif  
                        if (getLoadedProgram().find(external_program_) == std::string::npos)
                        {
                            int res = deactiveRunningProgram();
                            if (res == RES_FAILED_CALL)
                            {
                                disconnect();
                                reconnect();
                                closePopups();
                            }
                            else
                            {
                                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                if (requestLoadProgram() == RES_FAILED_CALL)
                                {
                                    disconnect();
                                    reconnect();
                                    closePopups();
                                }
                                else
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                    if (powerOff() == RES_FAILED_CALL)
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
                            if (!isProgramRunning())
                            {
                                if (requestPlay() == RES_FAILED_CALL)
                                {
                                    disconnect();
                                    reconnect();
                                    closePopups();
                                }
                            }
                            else
                            {
                                if (++programRunningCount >= 3)
                                {
                                    std::lock_guard<std::mutex> lock(mtx_);
                                    standby_ = true;
                                    cv_.notify_all();

                                    break;
                                }
                                else
                                {
                                    reconnect();
                                    closePopups();
                                }
                            }
                        }
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
#ifdef DEBUG
                std::cout << "mode outsides loopppppppppppp " << int(srvRobotMode.response.robot_mode.mode) << std::endl;
#endif
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
            ROS_WARN_STREAM("failed to set payload");
        }

        bool disconnected = false;
        while (!terminated_.load())
	    {
            if (disconnected)
            {
                reconnect();
                disconnected = closePopups() != RES_FAILED_CALL ? false : true;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            else
            {
                /// query the safty state
                bool protective = false;
                if (isProtective(protective) == RES_SUCCEED)
                {
                    if (protective)
                    {
                        // notify depended ones
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        standby_ = false;

                        // recover
                        int res = recoverFromProtective();
                        if (res != RES_FAILED_CALL)
                        {
                            // close popups
                            closePopups();

                            // reload and replay
                            if (requestPlay() != RES_FAILED_CALL)
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
                        whi_interfaces::WhiMotionState msg;
                        if (sub_moveit_cpp_state_)
                        {
                            msg.state = moveit_cpp_ready_ ?
                                whi_interfaces::WhiMotionState::STA_STANDBY : whi_interfaces::WhiMotionState::STA_BOOTING;
                        }
                        else
                        {
                            msg.state = whi_interfaces::WhiMotionState::STA_STANDBY;
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

        std::string service(service_prefix_ + prefix_dashboard_ + "get_loaded_program");
        auto clientLoaded = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::GetLoadedProgram>(service));
        ur_dashboard_msgs::GetLoadedProgram srvLoaded;
        if (clientLoaded->call(srvLoaded))
        {
            loaded.assign(srvLoaded.response.program_name);
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return loaded;
    }

    int UrRobotDriverBridge::requestLoadProgram()
    {
        // service load_program
        std::string service(service_prefix_ + prefix_dashboard_ + "load_program");
        auto clientLoadProgram = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::Load>(service));
        ur_dashboard_msgs::Load srvLoadProgram;
        srvLoadProgram.request.filename = external_program_;
        if (clientLoadProgram->call(srvLoadProgram))
        {
            if (srvLoadProgram.response.success)
            {
                ROS_INFO_STREAM("program " << srvLoadProgram.request.filename << " is loaded successfully");
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to execute service " << service << " to load program file "
                    << srvLoadProgram.request.filename);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::deactiveRunningProgram()
    {
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
                        return RES_SUCCEED;
                    }
                    else
                    {
                        ROS_WARN_STREAM("failed to execute service " << service << " to deactivate program");
                        return RES_FAILED_EXECUTE;
                    }
                }
            }
            else
            {
                ROS_INFO_STREAM("program is not running yet");
                return RES_SUCCEED;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::requestPlay()
    {
        // service play
        std::string service(service_prefix_ + prefix_dashboard_ + "play");
        auto clientPlay = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientPlay->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("UR is standby");
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::powerOn()
    {
        // service power_on
        std::string service(service_prefix_ + prefix_dashboard_ + "power_on");
        auto clientPowerOn = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientPowerOn->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("power on successfully");
                return RES_SUCCEED;
            }
            else
            {
                ROS_ERROR_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::powerOff()
    {
        // service power_off
        std::string service(service_prefix_ + prefix_dashboard_ + "power_off");
        auto clientPowerOff = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientPowerOff->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("power off successfully");
                return RES_SUCCEED;
            }
            else
            {
                ROS_ERROR_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::releaseBrake()
    {
        // service brake_release
        std::string service(service_prefix_ + prefix_dashboard_ + "brake_release");
        auto clientBrakeRelease = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientBrakeRelease->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("brake released successfully");
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::closePopups()
    {
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
                ROS_WARN_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
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
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::isProtective(bool& IsProtective)
    {
        // service get_safty_mode
        std::string service(service_prefix_ + prefix_dashboard_ + "get_safety_mode");
        auto clientSafetyMode = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::GetSafetyMode>(service));
        ur_dashboard_msgs::GetSafetyMode srvSafetyMode;
        if (clientSafetyMode->call(srvSafetyMode))
        {
            if (srvSafetyMode.response.safety_mode.mode == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP)
            {
                whi_interfaces::WhiMotionState msg;
                msg.state = whi_interfaces::WhiMotionState::STA_FAULT;
                pub_motion_state_->publish(msg);

                IsProtective = true;

                ROS_WARN_STREAM("UR entered protective stop state");
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

            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::recoverFromProtective()
    {
        // unlock protective
        std::string service(service_prefix_ + prefix_dashboard_ + "unlock_protective_stop");
        auto clientUnlockProtective = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientUnlockProtective->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("UR is recovered from protective state");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to execute service " << service);
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    bool UrRobotDriverBridge::isInRemote()
    {
        // service is_in_remote_control
        std::string service(service_prefix_ + prefix_dashboard_ + "is_in_remote_control");
        auto clientRemote = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::IsInRemoteControl>(service));
        ur_dashboard_msgs::IsInRemoteControl srvRemote;
        if (clientRemote->call(srvRemote) && srvRemote.response.success)
        {
            return srvRemote.response.in_remote_control;
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return false;
        }   
    }

    bool UrRobotDriverBridge::isProgramRunning()
    {
        // service program_state
        std::string service(service_prefix_ + prefix_dashboard_ + "program_state");
        auto client = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<ur_dashboard_msgs::GetProgramState>(service));
        ur_dashboard_msgs::GetProgramState srv;
        if (client->call(srv) && srv.response.success)
        {
            return srv.response.answer.find("PLAYING") == std::string::npos ? false : true;
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return false;
        }   
    }

    int UrRobotDriverBridge::disconnect()
    {
        std::string service(service_prefix_ + prefix_dashboard_ + "quit");
        auto clientQuit = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientQuit->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("disconnect successfully");
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to disconnect");
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    int UrRobotDriverBridge::reconnect()
    {
        std::string service(service_prefix_ + prefix_dashboard_ + "connect");
        auto clientConnect = std::make_unique<ros::ServiceClient>(
            node_handle_ns_free_->serviceClient<std_srvs::Trigger>(service));
        std_srvs::Trigger srv;
        if (clientConnect->call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO_STREAM("reconnect successfully");
                return RES_SUCCEED;
            }
            else
            {
                ROS_WARN_STREAM("failed to reconnect");
                return RES_FAILED_EXECUTE;
            }
        }
        else
        {
            ROS_ERROR_STREAM("failed to call service " << service);
            return RES_FAILED_CALL;
        }
    }

    bool UrRobotDriverBridge::setPayload()
    {
        double weight;
        node_handle_->param("payload_weight", weight, 1.0);
        std::vector<double> payload2Tcp;
        if (!node_handle_->getParam("payload_to_tcp", payload2Tcp))
        {
            payload2Tcp.resize(3);
        }
        std::string service(service_prefix_ + "set_payload");
        auto client = std::make_unique<ros::ServiceClient>(
            node_handle_->serviceClient<ur_msgs::SetPayload>(service));
        ur_msgs::SetPayload srv;
        srv.request.mass = weight;
        srv.request.center_of_gravity.x = payload2Tcp[0];
        srv.request.center_of_gravity.y = payload2Tcp[1];
        srv.request.center_of_gravity.z = payload2Tcp[2];
        if (client->call(srv))
        {
            if (!srv.response.success)
            {
                ROS_ERROR_STREAM("failed to execute service " << service);
            }
        }
        else
        {
            srv.response.success = false;
            ROS_ERROR_STREAM("failed to call service " << service);
        }

        return srv.response.success;
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

    void UrRobotDriverBridge::callbackMotionState(const std_msgs::Bool::ConstPtr& Msg)
    {
        moveit_cpp_ready_ = Msg->data;
    }
} // namespace whi_ur_robot_driver_bridge
