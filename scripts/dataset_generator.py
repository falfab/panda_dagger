#!/usr/bin/env python
import random
import os
import rospy

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
    rospy.init_node('dataset_generator_node')

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

    for i in range(50):
        print "Iteration: ", i
        
        dataset_handler = DatasetHandler(i)        
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
                
                dataset_handler.save()
                break
            print "compute master policy"
            moveit_handler.compute_master_policy(ring_handler)
                        
            dataset_handler.append((LAST_IMAGE,
                [ moveit_handler.delta_pose.pose.position.x,
                  moveit_handler.delta_pose.pose.position.y,
                  moveit_handler.delta_pose.pose.position.z] ) )
            
            pub_delta_controller.publish(moveit_handler.delta_pose)
            moveit_handler.update_target_pose()
            print "wait robot moving..."
            moveit_handler.wait(moveit_handler.target_pose)
            print "done."
            
        


if __name__ == '__main__':
    main()
