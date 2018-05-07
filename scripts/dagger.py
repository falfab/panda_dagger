import rospkg
import time
import csv

import numpy as np
import tensorflow as tf

import agent
import cv_bridge
import rospy

from moveit_handler import MoveItHandler
from ring_handler import RingHandler
from config_handler import ConfigHandler

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image, JointState

conf = ConfigHandler()

global LAST_IMAGE
LAST_IMAGE = None


def image_callback(image):
    global LAST_IMAGE
    LAST_IMAGE = image

def dagger():

    t = time.localtime()
    timestamp = time.strftime('%d_%b_%Y_%H_%M_%S', t)
    name = "experiment_"+timestamp+'.csv'
    csvfile =  open(name,'w')
    csvfile.write("# master iter {}, dagger iter {}, trained iter {}\n".format(
        conf.getint('Dagger', 'MasterIterations'),
        conf.getint('Dagger', 'DaggerIterations'),
        conf.getint('Dagger', 'TrainedIterations'))
        )

    csv_writer = csv.writer(csvfile)
    rospy.init_node('dagger_node')

    # initialize classes
    moveit_handler = MoveItHandler()
    ring_handler = RingHandler()
    bridge = cv_bridge.CvBridge()
    
    rospy.Subscriber("/vrep_ros_interface/image", Image, image_callback)
    rospy.Subscriber("/ring_current_position", Pose, ring_handler.update_ring_pose)

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
    iteration = 0
    start_time = time.time()

    csvfile.write("# master policy\n")
    csvfile.write("# n. iteration, time, n. step\n")
    # collect master policy
    for i in range(conf.getint('Dagger', 'MasterIterations')):
        print "Master iteration: ", i
        
        iteration = 0
        while not rospy.is_shutdown() and iteration < conf.getint('Dagger', 'MaxActions'):
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
                start_time = time.time()
                continue
            if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'MinStep'):
                csv_writer.writerow([i,time.time()-start_time,iteration])
                rospy.sleep(1)
                init_ring = True
                init_panda = True
                break
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
            moveit_handler.wait(moveit_handler.target_pose)     
            
    pass
    
    # do first training over partial data
    sess = tf.InteractiveSession()
    model = agent.Agent(sess=sess)
    model.train(images_all[:dataset_index], actions_all[:dataset_index], print_freq=5)
    
    model.save_model()
    
    # loop    
    csvfile.write("# training dagger\n")
    csvfile.write("# n. iteration, n. dagger iteration, success, time, n. step\n")

    init_ring = True
    init_panda = True
    start_time = time.time()
    for i in range(conf.getint('Dagger', 'DaggerIterations')):
        print "Dagger iteration: ", i
        
        # TODO for every iteration reward
        
        iteration = 0
        num_grasps = 0
        keep_grasping = True
        init_ring = True
        init_panda = True
        while not rospy.is_shutdown() and num_grasps < conf.getint('Dagger','TrainedIterations') and keep_grasping:
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
                start_time = time.time()
                continue

            if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'MinStep'):
                # succesfull trained
                print "grasp successfull"
                csv_writer.writerow([i,num_grasps,int(True),time.time()-start_time,iteration])
                rospy.sleep(1)
                init_ring = True
                init_panda = True
                num_grasps += 1
                iteration = 0
                continue

            if not ring_handler.is_ring_visible(moveit_handler.current_pose):
                # failed trained
                print "ring no more visible"
                csv_writer.writerow([i,num_grasps,int(False),time.time()-start_time,iteration])
                rospy.sleep(1)
                keep_grasping = False
                continue

            if iteration > conf.getint('Dagger', 'MaxActions'):
                # failed trained
                csv_writer.writerow([i,num_grasps,int(False),time.time()-start_time,iteration])
                print "Too many iterations for a single grasp"
                rospy.sleep(1)
                keep_grasping = False
                continue

            if moveit_handler.current_pose.pose.position.z < conf.getfloat('Goal','GoalHeight'):
                # failed trained
                csv_writer.writerow([i,num_grasps,int(False),time.time()-start_time,iteration])
                print "robot too close to the ground"
                rospy.sleep(1)
                keep_grasping = False
                continue

            # save master policy
            print "computing master policy and appending to dataset"
            moveit_handler.compute_master_policy(ring_handler)
            mat = bridge.imgmsg_to_cv2(
                LAST_IMAGE, desired_encoding='passthrough')

            images_all[dataset_index] = mat
            actions_all[dataset_index] = [
                moveit_handler.delta_pose.pose.position.x,
                moveit_handler.delta_pose.pose.position.y,
                moveit_handler.delta_pose.pose.position.z]

            dataset_index = dataset_index + 1
            iteration = iteration + 1

            moveit_handler.compute_trained_policy(model, mat)
            pub_delta_controller.publish(moveit_handler.delta_pose)

            moveit_handler.update_target_pose()
            moveit_handler.wait(moveit_handler.target_pose)


        # do training
        print "Retraining... ", i
        model.train(images_all[:dataset_index], actions_all[:dataset_index], print_freq=5)
        model.save_model()

    csvfile.write("# training dagger\n")
    csvfile.write("# n. iteration, success, time, n. step\n")

    success_count = 0
    failure_count = 0
    start_time = time.time()
    for count in range(100):
        num_grasps = 0
        init_ring = True
        init_panda = True
        keep_grasping = True
        iteration = 0
        while not rospy.is_shutdown() and keep_grasping:
            if init_panda:
                print "iteration",count
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
                start_time = time.time()
                continue
            if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'MinStep'):
                # succesfull trained
                print "grasp successfull"
                csv_writer.writerow([i,int(True),time.time()-start_time,iteration])
                rospy.sleep(1)
                keep_grasping = False
                success_count+=1
                continue
            if not ring_handler.is_ring_visible(moveit_handler.current_pose):
                # failed trained
                csv_writer.writerow([i,int(False),time.time()-start_time,iteration])
                print "ring no more visible"
                rospy.sleep(1)
                keep_grasping = False
                failure_count+=1
                continue
            if iteration > conf.getint('Dagger', 'MaxActions'):
                # failed trained
                csv_writer.writerow([i,int(False),time.time()-start_time,iteration])
                print "Too many iterations for a single grasp"
                rospy.sleep(1)
                keep_grasping = False
                failure_count+=1
                continue
            if moveit_handler.current_pose.pose.position.z < conf.getfloat('Goal','GoalHeight'):
                # failed trained
                csv_writer.writerow([i,int(False),time.time()-start_time,iteration])
                print "robot too close to the ground"
                rospy.sleep(1)
                keep_grasping = False
                failure_count+=1
                continue

            iteration = iteration + 1

            mat = bridge.imgmsg_to_cv2(
                LAST_IMAGE, desired_encoding='passthrough')

            moveit_handler.compute_trained_policy(model, mat)
            pub_delta_controller.publish(moveit_handler.delta_pose)

            moveit_handler.update_target_pose()
            moveit_handler.wait(moveit_handler.target_pose)


    print "success:", success_count
    print "failure:", failure_count
    csvfile.close()

# if __name__ == '__main__':
#     main()
