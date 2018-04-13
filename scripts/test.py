#!/usr/bin/env python
import random
import os
import rospy

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
os.chdir(os.path.dirname(os.path.realpath(__file__)))

conf = ConfigHandler()

global LAST_IMAGE
LAST_IMAGE = None


def image_callback(image):
    global LAST_IMAGE
    LAST_IMAGE = image


def main():

    sess = tf.InteractiveSession()
    model = agent.Agent(name='model_trained', sess=sess)
    model.load_model()

    rospy.init_node('dagger_test_node')

    moveit_handler = MoveItHandler()
    ring_handler = RingHandler()

    rospy.Subscriber("/vrep_ros_interface/image", Image, image_callback)

    pub_ring = rospy.Publisher(conf.get('Ring', 'PositionTopic'), Pose, queue_size=1)
    pub_joint_controller = rospy.Publisher('/dagger/joint_states', JointState, queue_size=1)
    pub_delta_controller = rospy.Publisher('/dagger/delta_pose', PoseStamped, queue_size=1)

    rospy.sleep(3)
    init_ring = True
    init_panda = True

    bridge = cv_bridge.CvBridge()
    for i in range(10000):
        print "Iteration: ", i
        
        #dataset_handler = DatasetHandler(i)        
        while not rospy.is_shutdown():
            if init_panda:
                # TODO: Da resettare target_joint_states alla posizione iniziale
                print "moving to: ", moveit_handler.target_joint_states
                pub_joint_controller.publish(moveit_handler.target_joint_states)
                init_panda = False
                print "wait robot moving..."
                moveit_handler.wait(moveit_handler.target_joint_states)
                print "done."
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
            if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'GraspHeight'):
                rospy.sleep(3)
                init_ring = True
                init_panda = True
                
                #dataset_handler.save()
                break
            print "compute master policy"
            #moveit_handler.compute_master_policy(ring_handler)

            mat = bridge.imgmsg_to_cv2(LAST_IMAGE, desired_encoding='passthrough')
            act = model.predict( np.reshape(mat,
                (1, agent.img_dim[0], agent.img_dim[1], agent.img_dim[2])))

            delta_pose = PoseStamped()
            delta_pose.pose.position = Point(act[0][0],act[0][1],act[0][2])
            delta_pose.pose.orientation.x = 0.923955
            delta_pose.pose.orientation.y = -0.382501
            delta_pose.pose.orientation.z = -0.000045
            delta_pose.pose.orientation.w = 0.000024

            pub_delta_controller.publish(delta_pose)

            moveit_handler.update_target_pose()
            print "wait robot moving..."
            moveit_handler.wait(moveit_handler.target_pose)
            print "done."

if __name__ == '__main__':
    main()
