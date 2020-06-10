#ifndef IK_FK_CALCULATOR
#define IK_FK_CALCULATOR

#include <iostream>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

// Message types
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveGroupActionGoal.h>


namespace panda_kinematics_solver{
class IkFkCalculator
{
    private:
        ros::NodeHandle _nh;
        robot_model::RobotModelPtr _kinematic_model;
        robot_state::RobotState* _kinematic_state;
        std::vector<double> _joint_values;
        std::vector<double> _goal_joint_values;
        std::vector<std::string> _joint_names;

        robot_state::JointModelGroup* _joint_model_group;

        void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
        void getJointValues();  
        Eigen::Isometry3d calculateFK();
        void calculateIK(Eigen::Isometry3d end_effector_state);
        void calculateJacobian();

    public:
        IkFkCalculator();
        ~IkFkCalculator();
};
}
#endif