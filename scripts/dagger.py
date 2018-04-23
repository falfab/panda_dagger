
import random
import os
import rospy
import cv_bridge
import agent

import tensorflow as tf
import numpy as np

from moveit_handler import MoveItHandler
from ring_handler import RingHandler
from dataset_handler import DatasetHandler
from config_handler import ConfigHandler

from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import Image, JointState

# Setting current working directory to the directory containing the file
os.chdir(os.path.dirname(os.path.realpath(__file__)))

conf = ConfigHandler()


global LAST_IMAGE
LAST_IMAGE = None


def image_callback(image):
    global LAST_IMAGE
    LAST_IMAGE = image


def main():
    rospy.init_node('dagger_node')

    # initialize classes
    moveit_handler = MoveItHandler()
    ring_handler = RingHandler()
    bridge = cv_bridge.CvBridge()
    
    rospy.Subscriber("/vrep_ros_interface/image", Image, image_callback)

    pub_ring = rospy.Publisher(conf.get('Ring', 'PositionTopic'), Pose, queue_size=1)
    pub_joint_controller = rospy.Publisher('/dagger/joint_states', JointState, queue_size=1)
    pub_delta_controller = rospy.Publisher('/dagger/delta_pose', PoseStamped, queue_size=1)
    
    # dataset initialization
    images_all = np.zeros( (conf.getint('Dagger', 'MaxArrayLen'), agent.img_dim[0], agent.img_dim[1], agent.img_dim[2]) )
    actions_all = np.zeros( (conf.getint('Dagger', 'MaxArrayLen'), agent.n_action) )
    dataset_index = 0
    
    rospy.sleep(1)
    init_ring = True
    init_panda = True
    
    # collect master policy
    for i in range(conf.getint('Dagger', 'MasterIterations')):
        print "Master iteration: ", i
        
        iteration = 0
        while not rospy.is_shutdown() and iteration < conf.getint('Dagger', 'MaxActions'):
            if init_panda:
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
                rospy.sleep(1)
                init_ring = True
                init_panda = True
                break
            print "compute master policy"
            moveit_handler.compute_master_policy(ring_handler)
            
            mat = bridge.imgmsg_to_cv2(LAST_IMAGE, desired_encoding='passthrough')
            images_all[dataset_index] = mat
            actions_all[dataset_index] = [ moveit_handler.delta_pose.pose.position.x,
                                           moveit_handler.delta_pose.pose.position.y,
                                           moveit_handler.delta_pose.pose.position.z ]
            dataset_index = dataset_index + 1
            iteration = iteration + 1
            
            pub_delta_controller.publish(moveit_handler.delta_pose)
            moveit_handler.update_target_pose()
            print "wait robot moving..."
            moveit_handler.wait(moveit_handler.target_pose)
            print "done."            
            
    pass
    
    # do first training over partial data
    sess = tf.InteractiveSession()
    model = agent.Agent(sess=sess)
    model.train(images_all[:dataset_index], actions_all[:dataset_index], print_freq=5)
    
    model.save_model()
    
    # loop    
    init_ring = True
    init_panda = True
    for i in range(conf.getint('Dagger', 'DaggerIterations')):
        print "Dagger iteration: ", i
        
        
        # TODO for every iteration reward
        
        iteration = 0
        while not rospy.is_shutdown() and iteration < conf.getint('Dagger', 'MaxActions'):
            if init_panda:
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
            if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'GraspHeight') \
                    or not ring_handler.is_ring_visible(moveit_handler.current_pose):                
                rospy.sleep(1)
                init_ring = True
                init_panda = True                
                break
            
            # save master policy
            print "computing master policy and appending to dataset"
            moveit_handler.compute_master_policy(ring_handler)            
            mat = bridge.imgmsg_to_cv2(LAST_IMAGE, desired_encoding='passthrough')
            images_all[dataset_index] = mat
            actions_all[dataset_index] = [ moveit_handler.delta_pose.pose.position.x,
                                           moveit_handler.delta_pose.pose.position.y,
                                           moveit_handler.delta_pose.pose.position.z ]
            dataset_index = dataset_index + 1
            iteration = iteration + 1
            

            print "compute trained policy"
            moveit_handler.compute_trained_policy(model,mat)
            pub_delta_controller.publish(moveit_handler.delta_pose)

            moveit_handler.update_target_pose()
            print "wait robot moving..."
            moveit_handler.wait(moveit_handler.target_pose)
            print "done."    
    
        # do training
        print "Retraining... ", i
        model.train(images_all[:dataset_index], actions_all[:dataset_index])
        model.save_model()
    

if __name__ == '__main__':
    main()









