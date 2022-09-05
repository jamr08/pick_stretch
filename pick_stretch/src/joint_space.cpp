/*
This file allows the stretch to pick a coke can from a table using hard coded location and joint commands for the planning group. The task is broken into the following steps:
1) the arm lift goes up
2) the wrist rotates
3) the arm stretches out
4) the gripper opens
5) the gripper grabs the can i.e gripper closes
6)the arm recoils 

The code is built from sample pick and place tutorials

https://moveit.readthedocs.io/en/latest/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html#planning-to-a-pose-goal
*/


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stretch_move_group_coke");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "stretch_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "stretch_gripper";
    static const std::string PLANNING_GROUP_HEAD = "stretch_head";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    moveit::planning_interface::MoveGroupInterface move_group_interface_head(PLANNING_GROUP_HEAD);

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    
    move_group_interface_arm.setGoalOrientationTolerance(0.00001);
    move_group_interface_arm.setGoalPositionTolerance(0.00001);
    
    move_group_interface_gripper.setGoalOrientationTolerance(0.00001);
    move_group_interface_gripper.setGoalPositionTolerance(0.00001);
    
    const robot_state::JointModelGroup *joint_model_group =  move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    const robot_state::JointModelGroup *joint_model_group_gripper =  move_group_interface_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    
    
    // 1. Move the lift to home pose
    moveit::core::RobotStatePtr current_state = move_group_interface_arm.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    ROS_INFO_NAMED("picker", "%ld", joint_group_positions.size());
    joint_group_positions[0] = 0.20000;  // meters
    joint_group_positions[1] = 0.00000;
    joint_group_positions[2] = 0.0000;
    joint_group_positions[3] = 0.0000;
    joint_group_positions[4] = 0.0000;
    joint_group_positions[5] = 3.142; //rad
    move_group_interface_arm.setJointValueTarget(joint_group_positions);
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();


    // 2. Raise the lift
    current_state = move_group_interface_arm.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 1.080;  // meters
    move_group_interface_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    
     // 3. Position the lift
    current_state = move_group_interface_arm.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 1.085;  // meters
    joint_group_positions[1] = 0.127;
    joint_group_positions[2] = 0.127;
    joint_group_positions[3] = 0.100;
    joint_group_positions[4] = 0.105;
    joint_group_positions[5] = -0.22; //rad
    move_group_interface_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    
        
    // 4. Open the gripper
    current_state = move_group_interface_gripper.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);
    ROS_INFO_NAMED("picker", "%ld", joint_group_positions.size());
    ROS_INFO_NAMED("picker", "right: %f", joint_group_positions[0]);
    ROS_INFO_NAMED("picker", "left: %f", joint_group_positions[1]);
    joint_group_positions[0] = 0.55;  // deg
    joint_group_positions[1] = 0.55;
    move_group_interface_gripper.setJointValueTarget(joint_group_positions);
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();
    
    
        
    // 5. Lower the arm
    current_state = move_group_interface_arm.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 1.045;  // meters
    joint_group_positions[1] = 0.127;
    joint_group_positions[2] = 0.127;
    joint_group_positions[3] = 0.100;
    joint_group_positions[4] = 0.105;
    joint_group_positions[5] = -0.22; //rad
    move_group_interface_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
        // 6. Close the gripper
    current_state = move_group_interface_gripper.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);
    ROS_INFO_NAMED("picker", "%ld", joint_group_positions.size());
    ROS_INFO_NAMED("picker", "right: %f", joint_group_positions[0]);
    ROS_INFO_NAMED("picker", "left: %f", joint_group_positions[1]);
    joint_group_positions[0] = 0.06;  // deg
    joint_group_positions[1] = 0.10;
    move_group_interface_gripper.setJointValueTarget(joint_group_positions);
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();
    
        // 7. Raise the lift
    current_state = move_group_interface_arm.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 1.080;  // meters
    move_group_interface_arm.setJointValueTarget(joint_group_positions);
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    
    
    


  ros::shutdown();
  return 0;
}
