#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

moveit::planning_interface::MoveGroupInterface* move_group;

void joint_states_callback(sensor_msgs::JointState joint_states)
{
  ROS_INFO("RECEIVED JOINT STATES");
  move_group->setJointValueTarget(joint_states.position);
  move_group->move();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "panda_joint_controller");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string joints_topic;
  nh.param<std::string>("joint_states_topic", joints_topic, "/dagger/joint_states");

  ros::Subscriber s = nh.subscribe<sensor_msgs::JointState>(joints_topic.c_str(), 1000, joint_states_callback);

  static const std::string PLANNING_GROUP = "panda_arm";

  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  ros::spin();
  ros::shutdown();

  delete move_group;

  return 0;
}