// Copyright (c) 2016, Toyota Research Institute. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_ros.h"

using namespace robotiq_3f_gripper_control;

Robotiq3FGripperROS::Robotiq3FGripperROS(ros::NodeHandle &nh, boost::shared_ptr<Robotiq3FGripperAPI> driver, std::vector<std::string> joint_names, ros::Duration desired_update_freq)
    :nh_(nh)
    ,driver_(driver)
    ,desired_update_freq_(desired_update_freq)
{
    //! advertise services
    init_srv_ = nh_.advertiseService("init", &Robotiq3FGripperROS::handleInit, this);
    reset_srv_ = nh_.advertiseService("reset", &Robotiq3FGripperROS::handleReset, this);
    halt_srv_ = nh_.advertiseService("halt", &Robotiq3FGripperROS::handleHalt, this);
    emerg_release_srv_ = nh_.advertiseService("emergency_release", &Robotiq3FGripperROS::handleEmergRelease, this);
    shutdown_srv_ = nh_.advertiseService("shutdown", &Robotiq3FGripperROS::handleShutdown, this);
    open_srv_ = nh_.advertiseService("open", &Robotiq3FGripperROS::handleOpen, this);
    close_srv_ = nh_.advertiseService("close", &Robotiq3FGripperROS::handleClose, this);

    //! advertise topics
    input_status_pub_ = nh.advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("input", 10);

    //! subscribers
    output_sub_ = nh.subscribe("output", 10, &Robotiq3FGripperROS::handleRawCmd, this);

    //! setup dynamic reconfigure
    reconfigure_.reset(new ReconfigureServer(reconfigure_mutex_, nh_));
    ReconfigureServer::CallbackType f = boost::bind(&Robotiq3FGripperROS::handleReconfigure, this, _1, _2);
    reconfigure_->setCallback(f);

    if(joint_names.size() != 4)
    {
        ROS_FATAL("Joint name size must be 4");
    }
}

void Robotiq3FGripperROS::publish()
{
    driver_->getRaw(&input_status_msg_);
    input_status_pub_.publish(input_status_msg_);
}

bool Robotiq3FGripperROS::handleInit(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    // ROS_INFO_STREAM("entered handle_init " << std::this_thread::get_id());
    //! set controller state (INIT_ACTIVATION)
    driver_->setInitialization(INIT_ACTIVATION);
    //! wait for controller state (GRIPPER_READY)
    while(!driver_->isReady())
    {   
        // ROS_INFO_STREAM("waiting for ready " << std::this_thread::get_id());
        desired_update_freq_.sleep();
    }
    resp.message += "Init succeeded. ";
    //! set action mode to go (ACTION_GO)

    driver_->setInidividualControlMode(IND_CONTROL_ON, IND_CONTROL_OFF);

    driver_->setActionMode(ACTION_GO);
    //! wait for controller state (ACTION_GO)
    while(driver_->isHalted())
    {
        // ROS_INFO_STREAM("waiting for halt " << std::this_thread::get_id());
        desired_update_freq_.sleep();
    }
    
    driver_->setVelocity(255, 255, 255, 255);
    resp.success = true;
    resp.message += "Ready to command. ";

    return true;
}

bool Robotiq3FGripperROS::handleClose(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp){
    if(!driver_->isInitialized())
    {
        // ROS_WARN("Gripper not initialized. ");
        resp.success = false;
        resp.message = "Not initialized. ";
        return true;
    }

    driver_->setGraspingMode(GRASP_BASIC);
    driver_->setPosition(255, 255, 255, 122);
    driver_->setActionMode(ACTION_GO);
    resp.success = true;
    return true;
}

bool Robotiq3FGripperROS::handleOpen(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp){
    if(!driver_->isInitialized())
    {
        // ROS_WARN("Gripper not initialized. ");
        resp.success = false;
        resp.message = "Not initialized. ";
        return true;
    }

    driver_->setGraspingMode(GRASP_BASIC);
    driver_->setPosition(0, 0, 0, 122);
    driver_->setActionMode(ACTION_GO);
    resp.success = true;
    return true;
}

bool Robotiq3FGripperROS::handleReset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    // ROS_INFO("entered handle_reset");
    // ROS_INFO_STREAM("entered handle_reset " << std::this_thread::get_id());
    //! shutdown
    handleShutdown(req, resp);
    //! init
    handleInit(req, resp);
    return true;
}

bool Robotiq3FGripperROS::handleHalt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    // ROS_INFO("entered handle_halt");
    // ROS_INFO_STREAM("entered handle_halt " << std::this_thread::get_id());

    //! check for activation
    if(!driver_->isInitialized())
    {
        // ROS_WARN("Gripper not initialized. ");
        resp.success = false;
        resp.message = "Not initialized. ";
        return true;
    }
    //! set controller state (ACTION_STOP)
    driver_->setActionMode(ACTION_STOP);
    //! wait for controller state (ACTION_STOP)
    while(!driver_->isHalted())
    {
        // ROS_INFO_STREAM("waiting for halt ");
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Device halted. ";
    return true;
}

bool Robotiq3FGripperROS::handleEmergRelease(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    // ROS_INFO("entered handle_emerg_release");
    //! halt
    handleHalt(req, resp);
    //! set controller state (EMERGENCY_RELEASE_ENGAGED)
    driver_->setEmergencyRelease(EMERGENCY_RELEASE_ENGAGED);
    //! wait for controller state (ERROR_AUTOMATIC_RELEASE_COMPLETED)
    while(!driver_->isEmergReleaseComplete())
    {
        desired_update_freq_.sleep();
    }
    driver_->setEmergencyRelease(EMERGENCY_RELEASE_IDLE);
    resp.success = true;
    resp.message += "Emergency release complete. ";
    return true;
}

bool Robotiq3FGripperROS::handleShutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    // ROS_INFO("entered handle_shutdown");
    //! halt
    handleHalt(req, resp);
    //! set controller state (INIT_RESET)
    driver_->setInitialization(INIT_RESET);
    //! wait for controller state (INIT_RESET)
    while(driver_->isInitialized())
    {
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Shutdown complete. ";
    return true;

}

void Robotiq3FGripperROS::handleReconfigure(robotiq_3f_gripper_control::Robotiq3FGripperConfig &config, uint32_t level)
{
    // ROS_INFO("entered handle_reconfigure");
    driver_->setInidividualControlMode((robotiq::IndividualControl)config.ind_control_fingers, (robotiq::IndividualControl)config.ind_control_scissor);
    if (!config.ind_control_scissor)
    {
        driver_->setGraspingMode((robotiq::GraspingMode)config.mode);

        while(!driver_->isModeSet((robotiq::GraspingMode)config.mode))
        {
            desired_update_freq_.sleep();
            ROS_DEBUG_STREAM("waiting for mode "<<config.mode<<" to be set");
        }
    }
    //! @todo automatically switch controller based on mode?
    //! @todo add individual velocity and force control

    driver_->setVelocity(config.velocity, config.velocity, config.velocity, config.velocity);
    driver_->setForce(config.force, config.force, config.force, config.force);

    config_ = config;
}


void Robotiq3FGripperROS::updateConfig(const robotiq_3f_gripper_control::Robotiq3FGripperConfig &config)
{
    reconfigure_->updateConfig(config);
}


void Robotiq3FGripperROS::getCurrentConfig(robotiq_3f_gripper_control::Robotiq3FGripperConfig &config)
{
    config = config_;
}


void Robotiq3FGripperROS::handleRawCmd(const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput::ConstPtr &msg)
{
    ROS_INFO("entered handle_raw_cmd");
    driver_->setRaw(*msg);
}
