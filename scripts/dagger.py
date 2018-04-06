#!/usr/bin/env python


import math
import random
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point

TOLERANCE = 0.001
COEFF = 0.03  # TODO variable according to point distance

DEFAULT_RING_POSITION_TOPIC = '/ring_position'
RING_MIN_X = -0.2
RING_MAX_X = 0.15
RING_MIN_Y = -0.3
RING_MAX_Y = 0.3
RING_Z = 0.0014
RING_RADIUS = 0.13971 / 2


def get_valid_ring_position():
    return Point(RING_MIN_X + (random.random() * (RING_MAX_X - RING_MIN_X)),
                 RING_MIN_Y + (random.random() * (RING_MAX_Y - RING_MIN_Y)),
                 RING_Z
                 )


def get_grasp_point(ring_point):
    """    
    Returns grasp point given the circle center position
    (geometry_msgs.msg.Point)
    """
    return Point(
        ring_point.x,
        (ring_point.y - RING_RADIUS) if (ring_point.y > 0) else (ring_point.y + RING_RADIUS),
        ring_point.z + 0.13  # TODO
    )


def get_master_policy(robot_pose_stamped, ring_point):
    """
    Returns master policy for delta controller
    (a unit vector pointing the ring as a triple)
    """
    grasp_point = get_grasp_point(ring_point)
    d_x = grasp_point.x - robot_pose_stamped.pose.position.x
    d_y = grasp_point.y - robot_pose_stamped.pose.position.y
    d_z = grasp_point.z - robot_pose_stamped.pose.position.z

    if abs(d_x) > TOLERANCE:
        alpha_yx = math.atan2(d_y, d_x)  # b/a
        alpha_zx = math.atan2(d_z, d_x)

        return Point(COEFF * math.cos(alpha_yx), COEFF * math.sin(alpha_yx), COEFF * math.sin(alpha_zx))
    elif abs(d_y) > TOLERANCE:
        alpha_zy = math.atan2(d_z, d_y)
        return Point(0., COEFF * math.cos(alpha_zy), COEFF * math.sin(alpha_zy))
    else:
        return Point(0., 0., COEFF * -1.)


def main():
    rospy.init_node('dagger_test_node')
    random.seed(rospy.get_time())

    # set ring
    pub_ring = rospy.Publisher(DEFAULT_RING_POSITION_TOPIC, Pose, queue_size=1)
    ring_pos = get_valid_ring_position()
    ring_pose = Pose()
    ring_pose.position = ring_pos
    pub_ring.publish(ring_pose)

    # delta between vrep and rviz on x of 0.5!!
    ring_pose.position.x += 0.5

    # home pose
    # (x: 0.307049, y: 0.000035, z: 0.590206, x1: 0.923955, y1: -0.382501, z1: -0.000045, w1: 0.000024)
    start_pose_stamped = PoseStamped()
    start_pose_stamped.pose.position.x = 0.307049
    start_pose_stamped.pose.position.y = 0.000035
    start_pose_stamped.pose.position.z = 0.590206
    start_pose_stamped.pose.orientation.x = 0.923955
    start_pose_stamped.pose.orientation.y = -0.382501
    start_pose_stamped.pose.orientation.z = -0.000045
    start_pose_stamped.pose.orientation.w = 0.000024
    current_pose_stamped = start_pose_stamped

    # set robot to start pose!
    pub_controller = rospy.Publisher('/dagger/pose', PoseStamped, queue_size=5)
    pub_controller.publish(start_pose_stamped)

    # delta robot
    pub_delta_controller = rospy.Publisher('/dagger/delta_pose', PoseStamped, queue_size=5)
    delta_pose = PoseStamped()
    delta_pose.pose.position = Point(0., 0., 0.)
    delta_pose.pose.orientation = start_pose_stamped.pose.orientation

    rospy.sleep(3)

    while not rospy.is_shutdown():
        new_point = get_master_policy(current_pose_stamped, ring_pos)

        delta_pose.pose.position = new_point

        current_pose_stamped.pose.position.x = current_pose_stamped.pose.position.x + new_point.x
        current_pose_stamped.pose.position.y = current_pose_stamped.pose.position.y + new_point.y
        current_pose_stamped.pose.position.z = current_pose_stamped.pose.position.z + new_point.z

        # do delta movement
        pub_delta_controller.publish(delta_pose)

        rospy.sleep(1)
    pass


if __name__ == '__main__':
    main()
