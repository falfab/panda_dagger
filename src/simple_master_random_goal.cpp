
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
    ros::init(argc, argv, "simple_master_random_goal");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped randomPose = move_group.getRandomPose();
    move_group.setPoseTarget(randomPose);
        
    ROS_INFO_NAMED("target_random","Random target: position(%f,%f,%f) orientation(%f,%f,%f,%f)", randomPose.pose.position.x, randomPose.pose.position.y, randomPose.pose.position.z,
    randomPose.pose.orientation.x, randomPose.pose.orientation.y, randomPose.pose.orientation.z, randomPose.pose.orientation.w
    
    );

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // planning
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Moving the "real" robot
    move_group.move();

    ros::shutdown();
    return 0;
}
