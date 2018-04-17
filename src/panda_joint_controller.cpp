#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <std_msgs/Bool.h>

moveit::planning_interface::MoveGroupInterface* move_group;

ros::Publisher planning_result_pub;  

void joint_states_callback(sensor_msgs::JointState joint_states)
{
    ROS_INFO_NAMED("dagger", "Joint_states_callback");
    move_group->setJointValueTarget(joint_states.position);
    move_group->move();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    std_msgs::Bool result_msg;
    result_msg.data = success;
    planning_result_pub.publish(result_msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "panda_joint_controller");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::string pose_topic;
  nh.param<std::string>("joint_states_topic", pose_topic, "/dagger/joint_states");
  ros::Subscriber s = nh.subscribe<sensor_msgs::JointState>(pose_topic.c_str(), 1000, joint_states_callback);

  std::string joint_planning_result_topic;
  nh.param<std::string>("joint_planning_result_topic", joint_planning_result_topic, "/dagger/planning_result/joint");
  planning_result_pub = nh.advertise<std_msgs::Bool>(joint_planning_result_topic.c_str(), 1000);

  static const std::string PLANNING_GROUP = "panda_arm";
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  ros::spin();
  ros::shutdown();

  delete move_group;

  return 0;
}