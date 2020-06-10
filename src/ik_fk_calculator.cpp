#include "panda_kinematics_solver/ik_fk_calculator.h"

namespace panda_kinematics_solver{

IkFkCalculator::IkFkCalculator(){
    // RobotModelLoader
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    _kinematic_model = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", _kinematic_model->getModelFrame().c_str());

    // We construct a RobotState that maintains the configuration of the robot
    // Joint states are set to default and then get a JointModelGroup, which represents the robot model for the panda_arm_hand group of the robot
    _kinematic_state = new robot_state::RobotState(_kinematic_model);
    _kinematic_state->setToDefaultValues(); // Set the joints in group to the position name defined in the SRDF
    _joint_model_group = _kinematic_model->getJointModelGroup("panda_arm");

    _joint_names = _joint_model_group->getVariableNames();

     getJointValues(); // from /joint_states ROS topic

    _kinematic_state->setJointGroupPositions(_joint_model_group, _joint_values);

    ROS_INFO_STREAM("Current state is " << (_kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

     Eigen::Isometry3d end_effector_state = calculateFK();

     calculateIK(end_effector_state);

     calculateJacobian();
}

IkFkCalculator::~IkFkCalculator(){
    delete _kinematic_state;
    delete _joint_model_group;
}

void IkFkCalculator::getJointValues(){
    sensor_msgs::JointState js;
    try {
        js = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", _nh));
    }
    catch(const std::exception& e){
    ROS_WARN("Message from /joint_states not received\n");
    }

    _joint_values.clear();

    _joint_values.push_back(js.position[2]); // panda_joint1
    _joint_values.push_back(js.position[3]); // panda_joint2
    _joint_values.push_back(js.position[4]); // panda_joint3
    _joint_values.push_back(js.position[5]); // panda_joint4
    _joint_values.push_back(js.position[6]); // panda_joint5
    _joint_values.push_back(js.position[7]); // panda_joint6
    _joint_values.push_back(js.position[8]); // panda_joint7
    _joint_values.push_back(js.position[0]); // panda_finger_joint1
    _joint_values.push_back(js.position[1]); // panda_finger_joint2

    ROS_INFO("\nCurrent set of joint values:");

    for (std::size_t i=0; i<_joint_names.size(); ++i){
        ROS_INFO("Joint %s: %f", _joint_names[i].c_str(), _joint_values[i]);
    }
}

Eigen::Isometry3d IkFkCalculator::calculateFK(){
    ROS_INFO_STREAM("\n\n**Solving forward kinematics**\n");

    // _kinematic_state->setToRandomPositions(_joint_model_group);
    Eigen::Isometry3d end_effector_state = _kinematic_state->getGlobalLinkTransform("panda_link8");

    ROS_INFO_STREAM("End-effector pose (in the model frame) : ");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    return end_effector_state;
}

void IkFkCalculator::calculateIK(Eigen::Isometry3d end_effector_state){
    ROS_INFO_STREAM("\n\n**Solving inverse kinematics**\n");

    double timeout = 0.1;
    bool found_ik = _kinematic_state->setFromIK(_joint_model_group, end_effector_state, timeout);

    if (found_ik){
        _kinematic_state->copyJointGroupPositions(_joint_model_group, _joint_values);
        for (std::size_t i=0; i<_joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", _joint_names[i].c_str(), _joint_values[i]);
        }
    }
    else{
        ROS_INFO("Did not find IK solution\n");
    }
}

void IkFkCalculator::calculateJacobian(){
    // Each column in the Jacobian matrix represents the effect on end-effector velocities due to variation in each joint velocity.
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    _kinematic_state->getJacobian(_joint_model_group, _kinematic_state->getLinkModel(_joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
    ROS_INFO_STREAM("\n\n**Jacobian** \n" << jacobian << "\n");
}


} // namespace panda_kinematics_solver

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_fk_calculator");

    panda_kinematics_solver::IkFkCalculator calc;

    ros::spin();
    return 0;
}
