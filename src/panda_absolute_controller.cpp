
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <iostream>

moveit::planning_interface::MoveGroupInterface* move_group;

void pose_callback(const geometry_msgs::PoseStampedConstPtr& pose)
{
  move_group->setPoseTarget(pose->pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("dagger", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group->move();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_absolute_controller");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string pose_topic;
  node_handle.param<std::string>("pose_topic", pose_topic, "/dagger/pose");

  ros::Subscriber s = node_handle.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1000, pose_callback);

  static const std::string PLANNING_GROUP = "panda_arm";

  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  ros::spin();
  ros::shutdown();

  delete move_group;

  return 0;
}
