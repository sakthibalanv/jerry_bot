#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <jerry_bot_driver/jerry_bot_hw_driver.h>

namespace jerry_bot
{
    jerryBotHWInterface::jerryBotHWInterface(ros::NodeHandle &nh): nh_(nh)
    {
        uc_statesReceived.data.push_back(0);
        uc_statesReceived.data.push_back(0);
        uc_statesReceived.data.push_back(0);
        uc_statesReceived.data.push_back(0);
        pub_uccmds_ = nh.advertise<jerry_bot_driver::uc_states>("uc_cmds", 1);
        sub_ucstates_ = nh.subscribe("/jerry_bot/uc_states", 1, &jerryBotHWInterface::uc_callback, this);
        init();
    }

    jerryBotHWInterface::~jerryBotHWInterface()
    {

    }

    void jerryBotHWInterface::uc_callback(const jerry_bot_driver::uc_states::ConstPtr& msg)
    {
        uc_statesReceived.data[0] = msg->data[0];
        uc_statesReceived.data[1] = msg->data[1];
        uc_statesReceived.data[2] = msg->data[2];
        uc_statesReceived.data[3] = msg->data[3];
    }


    void jerryBotHWInterface::init()
    {
        joint_position_[0] = 0;
        joint_position_[1] = 0;
        joint_velocity_[0] = 0;
        joint_velocity_[1] = 0;
        joint_effort_[0] = 0;
        joint_effort_[1] = 0;
        joint_velocity_command_[0] = 0;
        joint_velocity_command_[1] = 0;

        hardware_interface::JointStateHandle joint_state_handle_left("left_wheel_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
        joint_state_interface_.registerHandle(joint_state_handle_left);

        hardware_interface::JointStateHandle joint_state_handle_right("right_wheel_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
        joint_state_interface_.registerHandle(joint_state_handle_right);

        hardware_interface::JointHandle joint_handle_left(joint_state_handle_left, &joint_velocity_command_[0]);
        velocity_joint_interface_.registerHandle(joint_handle_left);

        hardware_interface::JointHandle joint_handle_right(joint_state_handle_right, &joint_velocity_command_[1]);
        velocity_joint_interface_.registerHandle(joint_handle_right);

        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
    }

    void jerryBotHWInterface::read()
    {
        //ROS_INFO("Read Started");
        joint_position_[0] = uc_statesReceived.data[0];
        joint_velocity_[0] = uc_statesReceived.data[1];
        joint_position_[1] = uc_statesReceived.data[2];
        joint_velocity_[1] = uc_statesReceived.data[3];
        //ROS_INFO("Read Sucess");
    }

    void jerryBotHWInterface::write()
    {
        //ROS_INFO("Write Started");
        // joint velocity commands from ros_control's RobotHW are in rad/s
        jerry_bot_driver::uc_states cmdsTouc;
        cmdsTouc.data.push_back(joint_velocity_command_[0]);
        cmdsTouc.data.push_back(joint_velocity_command_[1]);
        pub_uccmds_.publish(cmdsTouc);    
        //ROS_INFO("Write Sucess"); 
    }
};


int main(int argc, char **argv)
{
    // ROS_INFO("Joint State pos is : [%f] [%f]", msg->pos[0], msg->pos[1]);
    ROS_INFO("Init main jerrybot hw driver");
    ros::init(argc, argv, "jerry_bot_hw_interface");
    ros::NodeHandle nh;
    jerry_bot::jerryBotHWInterface myRobot(nh);
    controller_manager::ControllerManager cm(&myRobot);
    ROS_INFO("All objects created");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); // 10 Hz rate

    while (ros::ok())
    {
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        
        myRobot.read();
        //ROS_INFO("Update Started");
        cm.update(time, period);
        //ROS_INFO("Update Sucess");
        myRobot.write();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}