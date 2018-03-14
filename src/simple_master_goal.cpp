
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
    ros::init(argc, argv, "simple_master_goal");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    //ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    
    // position(-0.573274,0.497342,0.616409) orientation(-0.347796,-0.346663,0.191024,0.849925)
    
    target_pose1.orientation.x = -0.347796;
    target_pose1.orientation.y = -0.346663;
    target_pose1.orientation.z = 0.191024;
    target_pose1.orientation.w = 0.849925;
    target_pose1.position.x = -0.573274;
    target_pose1.position.y = 0.497342;
    target_pose1.position.z = 0.616409;
    
    /*target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.2;
    target_pose1.position.y = -0.07;
    target_pose1.position.z = 0.1;*/
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Moving the "real" robot
    move_group.move();

    ros::shutdown();
    return 0;
}
