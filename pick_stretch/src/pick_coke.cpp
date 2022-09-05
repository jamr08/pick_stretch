/*
This file allows the stretch to pick a coke can from a table using hard coded location and joint commands for the planning group. The task is broken into the following steps:
1) the arm lift goes up
2) the wrist rotates
3) the arm stretches out
4) the gripper opens
5) the gripper grabs the can i.e gripper closes
6)the arm recoils 

The code is built from sample pick and place tutorials
http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html
https://moveit.readthedocs.io/en/latest/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html#planning-to-a-pose-goal
https://moveit.readthedocs.io/en/latest/doc/trac_ik_tutorial.html
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
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    move_group_interface_arm.setGoalOrientationTolerance(0.000005);
    move_group_interface_arm.setGoalPositionTolerance(0.000005);
    
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    move_group_interface_gripper.setGoalOrientationTolerance(0.000005);
    move_group_interface_gripper.setGoalPositionTolerance(0.000005);


    // 1. Rotate gripper and Raise the lift
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("link_lift");
    ROS_INFO_NAMED("picker x", "%f",current_pose.pose.position.x);
    ROS_INFO_NAMED("picker y", "%f", current_pose.pose.position.y);
    ROS_INFO_NAMED("picker z", "%f", current_pose.pose.position.z);

    geometry_msgs::Pose target_pose1;
    
  
    //target_pose1.orientation = current_pose.pose.orientation;
 
    target_pose1.orientation.x = 3.1142e-06;
    target_pose1.orientation.y= 1.8357e-06; 
    target_pose1.orientation.z = -0.78235;
    target_pose1.orientation.w =  0.62284;
    
    target_pose1.position.x = -0.069979;
    target_pose1.position.y = -0.34731;
    target_pose1.position.z = 1.12;
    //move_group_interface_arm.setPoseTarget(target_pose1);
     move_group_interface_arm.setApproximateJointValueTarget(target_pose1,"link_grasp_center");
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 1:lifting (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    //2. Extend the arm
    current_pose = move_group_interface_arm.getCurrentPose("link_grasp_center");
    ROS_INFO_NAMED("picker x", "%f",current_pose.pose.position.x);
    ROS_INFO_NAMED("picker y", "%f", current_pose.pose.position.y);
    ROS_INFO_NAMED("picker z", "%f", current_pose.pose.position.z);
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = current_pose.pose.position.x;
    target_pose1.position.y = -0.8080;
    target_pose1.position.z = 1.12;
    move_group_interface_arm.setPoseTarget(target_pose1,"link_grasp_center");
    //move_group_interface_arm.setJointValueTarget(target_pose1,"link_grasp_center");
    // move_group_interface_arm.setApproximateJointValueTarget(target_pose1,"link_grasp_center");
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 2:extension (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    
    //3. Opent the gripper

     move_group_interface_gripper.setNamedTarget("open");
    success = ( move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 3:open gripper left (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();
    
    
    //4. Lower the lift
    current_pose = move_group_interface_arm.getCurrentPose("link_grasp_center");
    ROS_INFO_NAMED("picker x", "%f",current_pose.pose.position.x);
    ROS_INFO_NAMED("picker y", "%f", current_pose.pose.position.y);
    ROS_INFO_NAMED("picker z", "%f", current_pose.pose.position.z);
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = current_pose.pose.position.x;
    target_pose1.position.y = current_pose.pose.position.y;
    target_pose1.position.z = 1.00;
    move_group_interface_arm.setPoseTarget(target_pose1,"link_grasp_center");
    //move_group_interface_arm.setJointValueTarget(target_pose1,"link_grasp_center");
    // move_group_interface_arm.setApproximateJointValueTarget(target_pose1,"link_grasp_center");
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 4: lowering lift (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    /*
    //5. Close the gripper

     move_group_interface_gripper.setNamedTarget("closed");
    success = ( move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 5:close gripper left (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move(); 
    
        
    //4. Raise the lift
    current_pose = move_group_interface_arm.getCurrentPose("link_grasp_center");
    ROS_INFO_NAMED("picker x", "%f",current_pose.pose.position.x);
    ROS_INFO_NAMED("picker y", "%f", current_pose.pose.position.y);
    ROS_INFO_NAMED("picker z", "%f", current_pose.pose.position.z);
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = current_pose.pose.position.x;
    target_pose1.position.y = current_pose.pose.position.y;
    target_pose1.position.z = 1.12;
    move_group_interface_arm.setPoseTarget(target_pose1,"link_grasp_center");
    //move_group_interface_arm.setJointValueTarget(target_pose1,"link_grasp_center");
    // move_group_interface_arm.setApproximateJointValueTarget(target_pose1,"link_grasp_center");
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("picker", "Visualizing plan 6: raising lift (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    
    
    */
    


  ros::shutdown();
  return 0;
}
   
