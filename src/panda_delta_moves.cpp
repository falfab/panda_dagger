#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <iostream>

moveit::planning_interface::MoveGroupInterface *move_group;

void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose) 
{
    geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

    ROS_INFO_NAMED("dagger", "Current pose: (x: %lf, y: %lf, z: %lf)",
     current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z );

    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.orientation = current_pose.pose.orientation;
    target_pose.pose.position.x = current_pose.pose.position.x + pose->pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y + pose->pose.position.y;
    target_pose.pose.position.z = current_pose.pose.position.z + pose->pose.position.z;

    ROS_INFO_NAMED("dagger", "Target pose: (x: %lf, y: %lf, z: %lf, x1: %lf, y1: %lf, z1: %lf, w1: %lf)",
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z ,
    target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z,
    target_pose.pose.orientation.w
    );

    move_group->setPoseTarget(target_pose.pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("dagger", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group->move();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_delta_moves");
    ros::NodeHandle node_handle("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string pose_topic;
    node_handle.param<std::string>("pose_topic", pose_topic, "/dagger/delta_pose");

    ros::Subscriber s = node_handle.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1000, pose_callback);

    static const std::string PLANNING_GROUP = "panda_arm";

    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
            move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ros::spin();
    ros::shutdown();

    delete move_group;

    return 0;
}
