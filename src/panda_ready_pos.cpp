
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_ready_pos");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  geometry_msgs::Pose target_pose1;

  target_pose1.orientation.x = 0.923955;
  target_pose1.orientation.y = -0.382501;
  target_pose1.orientation.z = -0.000045;
  target_pose1.orientation.w = 0.000024;
  target_pose1.position.x = 0.307049;
  target_pose1.position.y = 0.000035;
  target_pose1.position.z = 0.590206;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  move_group.move();

  ros::shutdown();
  return 0;
}
