
#include <moveit/move_group_interface/move_group_interface.h>

#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_ready_pos");
  ros::NodeHandle nh;

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  std::map<std::string, double> joint_position = { { "panda_joint1", 0 },    { "panda_joint2", -0.785 },
                                                   { "panda_joint3", 0 },    { "panda_joint4", -2.356 },
                                                   { "panda_joint5", 0 },    { "panda_joint6", 1.571 },
                                                   { "panda_joint7", 0.785 } };
  move_group.setJointValueTarget(joint_position);
  move_group.move();

  // ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/dagger/joint_states", 1000);

  // sensor_msgs::JointState js;
  // js.name = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
  //             "panda_joint5", "panda_joint6", "panda_joint7" };
  // js.position = { 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };

  // while (ros::ok())
  // {
  //   pub.publish(js);
  // }

  ros::shutdown();
  return 0;
}
