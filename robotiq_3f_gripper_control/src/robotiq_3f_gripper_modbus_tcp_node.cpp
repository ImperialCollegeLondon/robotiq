#include <robotiq_3f_gripper_control/robotiq_3f_gripper_modbus_tcp_client.h>
#include <robotiq_3f_gripper_control/robotiq_3f_gripper_api.h>
#include <robotiq_3f_gripper_control/robotiq_3f_gripper_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

class GenericHWLoop
{
public:
    GenericHWLoop(
            const ros::NodeHandle& pub_nh, const ros::NodeHandle& prv_nh,
            boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperHWInterface> hardware_interface)
        : nh_(prv_nh), pub_nh(pub_nh), name_("generic_hw_control_loop"), hardware_interface_(hardware_interface)
    {
        ROS_DEBUG("creating loop");

        //! Create the controller manager
        controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), prv_nh));
        ROS_DEBUG("created controller manager");

        //! Load rosparams
        ros::NodeHandle rpsnh(nh_, name_);

        loop_hz_ = rpsnh.param<double>("loop_hz", 30);
        cycle_time_error_threshold_ = rpsnh.param<double>("cycle_time_error_threshold", 0.1);

        //! Get current time for use with first update
        clock_gettime(CLOCK_MONOTONIC, &last_time_);

        //! Start timer
        ros::Duration desired_update_freq_ = ros::Duration(1 / loop_hz_);
        non_realtime_loop_ = nh_.createTimer(desired_update_freq_, &GenericHWLoop::update, this);
        ROS_DEBUG("created timer");
    }

    /** \brief Timer event
     *         Note: we do not use the TimerEvent time difference because it does NOT guarantee that
     * the time source is
     *         strictly linearly increasing
     */
    void update(const ros::TimerEvent& e)
    {
        //! Get change in time
        clock_gettime(CLOCK_MONOTONIC, &current_time_);
        elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec
                                      + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
        last_time_ = current_time_;
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "GenericHWLoop","Sampled update loop with elapsed time " << elapsed_time_.toSec());

        //! Error check cycle time
        const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
        if (cycle_time_error > cycle_time_error_threshold_)
        {
            ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                  << cycle_time_error << ", cycle time: " << elapsed_time_
                                  << ", threshold: " << cycle_time_error_threshold_);
        }

        //! Input
        hardware_interface_->read(elapsed_time_);

        //! Control
        controller_manager_->update(ros::Time::now(), elapsed_time_);

        //! Output
        hardware_interface_->write(elapsed_time_);
    }

protected:
    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;
    ros::NodeHandle pub_nh;

    // Name of this class
    std::string name_;

    // Settings
    ros::Duration desired_update_freq_;
    double cycle_time_error_threshold_;

    // Timing
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    struct timespec last_time_;
    struct timespec current_time_;

    /** \brief ROS Controller Manager and Runner
     *
     * This class advertises a ROS interface for loading, unloading, starting, and
     * stopping ros_control-based controllers. It also serializes execution of all
     * running controllers in \ref update.
     */
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    /** \brief Abstract Hardware Interface for your robot */
    boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperHWInterface> hardware_interface_;
};

int main(int argc, char** argv)
{

    using robotiq_3f_gripper_control::Robotiq3FGripperModbusTCPClient;
    ros::init(argc, argv, "robotiq_3f_gripper_node");
    ros::NodeHandle nh;
    ros::NodeHandle prv_nh("~");

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Parameter names
    std::string host;
    int port;
    bool activate;

    prv_nh.param<std::string>("host", host, "192.168.0.0");
    prv_nh.param<int>("port", port, 1);
    prv_nh.param<bool>("activate", activate, true);



    // Create the hw client layer
    boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperModbusTCPClient> modbus_client
            (new robotiq_3f_gripper_control::Robotiq3FGripperModbusTCPClient(host, port));
    modbus_client->init(prv_nh);

    // Create the hw api layer
    boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperAPI> hw_api
            (new robotiq_3f_gripper_control::Robotiq3FGripperAPI(modbus_client));

    // Create the hardware interface layer
    boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperHWInterface> hw_interface
            (new robotiq_3f_gripper_control::Robotiq3FGripperHWInterface(prv_nh, hw_api));

    ROS_DEBUG("created hw interface");

    // Register interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    hw_interface->configure(joint_state_interface, position_joint_interface);
    hw_interface->registerInterface(&joint_state_interface);
    hw_interface->registerInterface(&position_joint_interface);

    ROS_DEBUG("registered control interfaces");

    // Start the control loop
    GenericHWLoop control_loop(nh, prv_nh, hw_interface);
    ROS_INFO("started");

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}

