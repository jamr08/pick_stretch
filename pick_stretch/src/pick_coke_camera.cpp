/*
This file allows the stretch to pick a coke can from a table using hard coded location and joint commands for the planning group. The task is broken into the following steps:
1) the arm lift goes up
2) the wrist rotates
3) the arm stretches out
4) the gripper opens
5) the gripper grabs the can i.e gripper closes
6)the arm recoils 

This builds on the pick_coke.cpp to allow for camera based object detection and collision.

The code is built from sample pick and place tutorials
http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html
https://moveit.readthedocs.io/en/latest/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html#planning-to-a-pose-goal
https://moveit.readthedocs.io/en/latest/doc/trac_ik_tutorial.html
*/


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "ros/ros.h"

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stretch_move_group_coke_collision");
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
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
        robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    // Add the object to be grasped (the suqare box) to the planning scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object.id = "blue_box";

    shape_msgs::SolidPrimitive primitive; // 0.082 0.0950 0.0750
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.6;
    primitive.dimensions[2] = 0.6;

    geometry_msgs::Pose box_pose; //0.3 0.8 1.04493 -1e-06 -2e-05 1e-06
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.8;
    box_pose.position.z = 1.045 - 1.21;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO_NAMED("picked", "Add an object into the world");

    ros::Duration(0.1).sleep();

    // Allow collisions between the gripper and the box to be able to grasp it
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
    acm.setEntry("blue_box", "link_gripper_fingertip_left", true);
    acm.setEntry("blue_box", "link_gripper_fingertip_right", true);
    std::cout << "\nAllowedCollisionMatrix:\n";
    acm.print(std::cout);
    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    planning_scene_interface.applyPlanningScene(diff_scene); 

    ros::Duration(0.1).sleep();


    

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("picked", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    move_group_interface_arm.setGoalOrientationTolerance(0.000005);
    move_group_interface_arm.setGoalPositionTolerance(0.000005);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    move_group_interface_gripper.setGoalOrientationTolerance(0.000005);
    move_group_interface_gripper.setGoalPositionTolerance(0.000005);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_head;
    move_group_interface_head.setGoalOrientationTolerance(0.000001);
    move_group_interface_head.setGoalPositionTolerance(0.000001);
    
    // 0. Rotate the camera
	geometry_msgs::PoseStamped current_pose;
	current_pose = move_group_interface_head.getCurrentPose("camera_color_optical_frame");
	ROS_INFO_NAMED("picker x", "%f", current_pose.pose.orientation.x);
        ROS_INFO_NAMED("picker y", "%f", current_pose.pose.orientation.y);
        ROS_INFO_NAMED("picker z", "%f", current_pose.pose.orientation.z);
	geometry_msgs::Pose target_pose1;
	//target_pose1.position = current_pose.pose.position;
	target_pose1.orientation.x = 0.52826;
        target_pose1.orientation.y= 0.470085; 
        target_pose1.orientation.z = -0.528213;
        target_pose1.orientation.w =  0.470057;
        
        target_pose1.position.x = -0.00464116;
        target_pose1.position.y = -0.0386679;
        target_pose1.position.z = 1.322239;
        move_group_interface_head.setJointValueTarget(target_pose1,"camera_color_optical_frame");
        bool success = (move_group_interface_head.plan(my_plan_head) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("picked", "Visualizing plan 0:head pan (pose goal) %s", success ? "" : "FAILED");
        move_group_interface_head.move();
	
	

    // 1. Rotate gripper and Raise the lift
    
    current_pose = move_group_interface_arm.getCurrentPose("link_lift");
    ROS_INFO_NAMED("picker x", "%f",current_pose.pose.position.x);
    ROS_INFO_NAMED("picker y", "%f", current_pose.pose.position.y);
    ROS_INFO_NAMED("picker z", "%f", current_pose.pose.position.z);

  
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
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
    move_group_interface_arm.setApproximateJointValueTarget(target_pose1,"link_grasp_center");
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
   
