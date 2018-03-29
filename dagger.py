#!/usr/bin/env python


import math
import time
import random
random.seed(15)

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point

TOLERANCE  =  0.0001
COEFF      =  1.

DEFAULT_RING_POSITION_TOPIC = '/ring_position'
RING_MIN_X  = -0.2
RING_MAX_X  =  0.15
RING_MIN_Y  = -0.3
RING_MAX_Y  =  0.3
RING_Z      =  0.0014
RING_RADIUS =  0.13971 / 2


def get_valid_ring_position():
    return Point(random.uniform(RING_MIN_X, RING_MAX_X), 
             random.uniform(RING_MIN_Y, RING_MAX_Y), 
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
            ring_point.z
           )


def get_master_policy(robot_pose_stamped, ring_point):
    """
    Returns master policy for delta controller
    (a unit vector pointing the ring as a triple)
    """
    grasp_point = get_grasp_point(ring_point)
    if math.abs(robot_pose_stamped.pose.position.x - grasp_point.x) < TOLERANCE:
        m_yx = (robot_pose_stamped.pose.position.y - grasp_point.y) / (robot_pose_stamped.pose.position.x - grasp_point.x)
        m_zx = (robot_pose_stamped.pose.position.z - grasp_point.z) / (robot_pose_stamped.pose.position.x - grasp_point.x)
        alpha_yx = math.atan(m_yx)  # b/a
        alpha_zx = math.atan(m_zx)
        
        return Point( COEFF * math.cos(alpha_yx), COEFF * math.sin(alpha_yx), COEFF * math.sin(alpha_zx))
    elif math.abs(robot_pose_stamped.pose.position.y - grasp_point.y) < TOLERANCE:        
        m_zy = (robot_pose_stamped.pose.position.z - grasp_point.z) / (robot_pose_stamped.pose.position.y - grasp_point.y)
        alpha_zy = math.atan(m_zy)
        return Point( 0., COEFF * math.cos(alpha_zy), COEFF * math.sin(alpha_zy))
    else:
        return Point(0., 0., COEFF * -1.)


def main(): 
    rospy.init_node('~')
    rate = rospy.Rate(1)  # 1 hz
    
    # set ring
    pub_ring = rospy.Publisher(DEFAULT_RING_POSITION_TOPIC, geometry_msgs.Pose)
    ring_pos = get_valid_ring_position()
    pub_ring.publish(ring_pos)
    
    time.sleep(1)
    
    # set robot to start pose!
    # TODO delta fra vrep e rviz sulla x di 0.5!!
    
    # home
    # (x: 0.307049, y: 0.000035, z: 0.590206, x1: 0.923955, y1: -0.382501, z1: -0.000045, w1: 0.000024
    start_pose_stamped = PoseStamped
    start_pose_stamped.header = Header
    
    currentposestamped = start_pose_stamped
    
    pub_controller = rospy.Publisher('/dagger/pose', geometry_msgs.PoseStamped)
    pub_controller.publish(start_pose_stamped)
    
    # delta robot
    pub_delta_controller = rospy.Publisher('/dagger/delta_pose', geometry_msgs.PoseStamped)
    delta_pose = PoseStamped
    # todo add header in qualche modo
    
    while not rospy.is_shutdown():
        new_point = get_master_policy(currentposestamped, ring_pos)
        
        delta_pose.pose.position.x = delta_pose.pose.position.x + new_point.x
        delta_pose.pose.position.y = delta_pose.pose.position.y + new_point.y
        delta_pose.pose.position.z = delta_pose.pose.position.z + new_point.z
        
        currentposestamped.pose.position.x = currentposestamped.pose.position.x + new_point.x
        currentposestamped.pose.position.y = currentposestamped.pose.position.y + new_point.y
        currentposestamped.pose.position.z = currentposestamped.pose.position.z + new_point.z
        
        pub_delta_controller.publish(delta_pose)
        
        rate.sleep()
    pass


if __name__ == '__main__':
    main()







