
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_goal_generator");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double period;
    node_handle.param<double>("period", period, 10.0);
    ros::Duration duration(period);

    std::string topic_pub;
    node_handle.param<std::string>("topic_pub", topic_pub, "/dagger/pose");
    
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>(topic_pub.c_str(), 1000);
    
    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    while(ros::ok())
    {
        geometry_msgs::PoseStamped randomPose = move_group.getRandomPose();
        pose_pub.publish(randomPose);
        ros::spinOnce();
        duration.sleep();
    }
   
    ros::shutdown();
    return 0;
}
