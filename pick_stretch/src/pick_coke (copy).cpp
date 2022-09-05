/*
This file allows the stretch to pick a coke can from a table using hard coded location and joint commands for the planning group. The task is broken into the following steps:
1) the arm lift goes up
2) the wrist rotates
3) the arm stretches out
4) the gripper opens
5) the gripper grabs the can i.e gripper closes
6)the arm recoils 

The code is built from sample pick and place tutorials
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
    move_group_interface_arm.setGoalOrientationTolerance(0.1);
    move_group_interface_arm.setGoalPositionTolerance(0.1);
    
  /*  
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move(); */

    // 2. Raise the lift
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("link_lift");
    ROS_INFO_NAMED("picker x", "%f",current_pose.pose.position.x);
    ROS_INFO_NAMED("picker y", "%f", current_pose.pose.position.y);
    ROS_INFO_NAMED("picker z", "%f", current_pose.pose.position.z);

    geometry_msgs::Pose target_pose1;
  
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = current_pose.pose.position.x;
    target_pose1.position.y = current_pose.pose.position.y;
    target_pose1.position.z = current_pose.pose.position.z +  0.877161;
    move_group_interface_arm.setPoseTarget(target_pose1);

    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("picker", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


    move_group_interface_arm.move();


  ros::shutdown();
  return 0;
}
