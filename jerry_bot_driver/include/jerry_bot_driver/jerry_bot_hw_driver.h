#ifndef JERRY_BOT_HW_DRIVER_H
#define JERRY_BOT_HW_DRIVER_H

#include <ros/ros.h>
#include <jerry_bot_driver/uc_states.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace jerry_bot
{
    class jerryBotHWInterface : public hardware_interface::RobotHW
    {
        public:
        jerryBotHWInterface(ros::NodeHandle &nh);
        ~jerryBotHWInterface();
        void init();
        void read();
        void write();

        protected:
        void uc_callback(const jerry_bot_driver::uc_states::ConstPtr& msg);

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        double joint_position_[2];
        double joint_velocity_[2];
        double joint_effort_[2];
        double joint_velocity_command_[2];

        ros::NodeHandle nh_;
        ros::Publisher pub_uccmds_;
        ros::Subscriber sub_ucstates_;

        jerry_bot_driver::uc_states uc_statesReceived;
    };
}

#endif