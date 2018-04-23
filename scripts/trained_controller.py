#!/usr/bin/env python
import random
import os
import rospy
from rospkg import RosPack

from moveit_handler import MoveItHandler
from ring_handler import RingHandler
from dataset_handler import DatasetHandler
from config_handler import ConfigHandler
from geometry_msgs.msg import PoseStamped, Pose, Point

from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import Image, JointState

import agent

import tensorflow as tf
import numpy as np

import cv_bridge

# Setting current working directory to the directory containing the file
os.chdir(RosPack().get_path('panda_dagger'))

conf = ConfigHandler()

global LAST_IMAGE
LAST_IMAGE = None


def image_callback(image):
    global LAST_IMAGE
    LAST_IMAGE = image


def trained_controller():

    sess = tf.InteractiveSession()
    model = agent.Agent(name='model', sess=sess)
    model.load_model()

    rospy.init_node('dagger_test_node')

    moveit_handler = MoveItHandler()
    ring_handler = RingHandler()

    rospy.Subscriber("/vrep_ros_interface/image", Image, image_callback)
    rospy.Subscriber("/ring_current_position", Pose, ring_handler.update_ring_pose)

    pub_ring = rospy.Publisher(conf.get('Ring', 'PositionTopic'), Pose, queue_size=1)
    pub_joint_controller = rospy.Publisher('/dagger/joint_states', JointState, queue_size=1)
    pub_delta_controller = rospy.Publisher('/dagger/delta_pose', PoseStamped, queue_size=1)

    rospy.sleep(3)
    init_ring = True
    init_panda = True

    bridge = cv_bridge.CvBridge()
    for i in range(5):
        print "Iteration: ", i
        
        #dataset_handler = DatasetHandler(i)        
        iteration = 0
        init_ring = True
        init_panda = True
        while not rospy.is_shutdown():
            if init_panda:
                print "moving to home position"
                pub_joint_controller.publish(moveit_handler.target_joint_states)
                init_panda = False
                moveit_handler.wait(moveit_handler.target_joint_states)
                continue
            if init_ring:
                print "setting ring to random pose"
                ring_handler.set_random_valid_pose()
                ring_pose = ring_handler.get_ring_pose()
                pub_ring.publish(ring_pose)
                # delta between vrep and rviz on x of 0.5!!
                ring_handler.ring_coordinate.x += 0.5
                init_ring = False
                rospy.sleep(1)
                continue

            if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'MinStep'):
                # succesfull trained
                print "grasp successfull"
                rospy.sleep(1)
                iteration = 0
                break

            if not ring_handler.is_ring_visible(moveit_handler.current_pose):
                # failed trained
                print "ring no more visible"
                rospy.sleep(1)
                break

            if iteration > conf.getint('Dagger', 'MaxActions'):
                # failed trained
                print "Too many iterations for a single grasp"
                rospy.sleep(1)
                break

            if moveit_handler.current_pose.pose.position.z < conf.getfloat('Goal','GoalHeight'):
                # failed trained
                print "robot too close to the ground"
                rospy.sleep(1)
                break

            mat = bridge.imgmsg_to_cv2(LAST_IMAGE, desired_encoding='passthrough')

            # print "compute trained policy"
            moveit_handler.compute_trained_policy(model,mat)

            pub_delta_controller.publish(moveit_handler.delta_pose)

            moveit_handler.update_target_pose()
            # print "wait robot moving..."
            moveit_handler.wait(moveit_handler.target_pose)
            # print "done."

# if __name__ == '__main__':
#     main()
