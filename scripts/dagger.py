#!/usr/bin/env python
import math
import pickle
import random
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image

TOLERANCE = 0.001
TOLERANCE_Z = 0.01
POSITION_TOLERANCE = 0.001
MOVEMENT_FRACTION = 1.0 / 20

DEFAULT_RING_POSITION_TOPIC = '/ring_position'
RING_MIN_X = -0.2
RING_MAX_X = 0.15
RING_MIN_Y = -0.3
RING_MAX_Y = 0.3
RING_Z = 0.0014
RING_RADIUS = 0.13971 / 2

GOAL_HEIGHT = RING_Z + 0.15
GRASP_DISTANCE = 0.009  # The distance to begin open pinze

global LAST_IMAGE
LAST_IMAGE = None


def get_coeff(robot_pose_stamped, ring_point):
    robot_point = robot_pose_stamped.pose.position

    d_x = robot_point.x - ring_point.x
    d_y = robot_point.y - ring_point.y
    d_z = robot_point.z - ring_point.z

    d_xy = math.sqrt(d_x**2 + d_y**2)
    distance = math.sqrt(d_xy**2 + d_z**2)

    return distance * MOVEMENT_FRACTION


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
        (ring_point.y - RING_RADIUS) if (ring_point.y >
                                         0) else (ring_point.y + RING_RADIUS),
        GOAL_HEIGHT
    )


def equal_poses(pose1, pose2):
    val = abs(pose1.position.x - pose2.position.x) < POSITION_TOLERANCE and \
        abs(pose1.position.y - pose2.position.y) < POSITION_TOLERANCE and \
        abs(pose1.position.z - pose2.position.z) < POSITION_TOLERANCE
    return val


def equal_joint_states(joint_array1, joint_array2):
    if len(joint_array1) != len(joint_array2):
        return False
    for (index, joint1) in enumerate(joint_array1):
        if abs(joint1 - joint_array2[index]) > POSITION_TOLERANCE:
            return False
    return True


def wait_move(target, move_group):
    if isinstance(target, PoseStamped) or isinstance(target, Pose):
        robot_real_pose_stamped = move_group.get_current_pose()
        while not equal_poses(robot_real_pose_stamped.pose, target.pose):
            robot_real_pose_stamped = move_group.get_current_pose()
            rospy.sleep(0.1)
    elif isinstance(target, JointState):
        real_robot_joint_state = move_group.get_current_joint_values()
        while not equal_joint_states(real_robot_joint_state, target.position):
            real_robot_joint_state = move_group.get_current_joint_values()
            rospy.sleep(0.1)
    else:
        raise ValueError(
            'target object passed to wait_move() is neither a pose or jointstates')


def get_master_policy(robot_pose_stamped, ring_point):
    """
    Returns master policy for delta controller
    (a unit vector pointing the ring as a triple)
    """
    coeff = get_coeff(robot_pose_stamped, ring_point)

    grasp_point = get_grasp_point(ring_point)
    d_x = grasp_point.x - robot_pose_stamped.pose.position.x
    d_y = grasp_point.y - robot_pose_stamped.pose.position.y
    d_z = grasp_point.z - robot_pose_stamped.pose.position.z

    if abs(d_x) > TOLERANCE:
        alpha_yx = math.atan2(d_y, d_x)  # b/a
        alpha_zx = math.atan2(d_z, d_x)
        return Point(coeff * math.cos(alpha_yx), coeff * math.sin(alpha_yx), coeff * math.sin(alpha_zx))
    elif abs(d_y) > TOLERANCE:
        alpha_zy = math.atan2(d_z, d_y)
        return Point(0., coeff * math.cos(alpha_zy), coeff * math.sin(alpha_zy))
    elif abs(d_z) > TOLERANCE_Z:
        return Point(0., 0., coeff * -1.)
    else:
        return Point(0., 0., 0.)


def init_delta_pose_stamped():
    delta_pose = PoseStamped()
    delta_pose.pose.position = Point(0., 0., 0.)
    delta_pose.pose.orientation.x = 0.923955
    delta_pose.pose.orientation.y = -0.382501
    delta_pose.pose.orientation.z = -0.000045
    delta_pose.pose.orientation.w = 0.000024
    return delta_pose


def init_joint_states():
    joint_states = JointState()
    joint_states.position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    return joint_states


def image_callback(image_ptr):
    global LAST_IMAGE
    LAST_IMAGE = image_ptr.data


def main():
    rospy.init_node('dagger_test_node')
    random.seed(rospy.get_time())

    move_group = moveit_commander.MoveGroupCommander('panda_arm')

    rospy.Subscriber("/vrep_ros_interface/image", Image, image_callback)

    # set ring
    pub_ring = rospy.Publisher(DEFAULT_RING_POSITION_TOPIC, Pose, queue_size=1)

    # set robot to start pose!
    start_joint_states = init_joint_states()
    pub_joint_controller = rospy.Publisher(
        '/dagger/joint_states', JointState, queue_size=1)

    # delta robot
    pub_delta_controller = rospy.Publisher(
        '/dagger/delta_pose', PoseStamped, queue_size=1)
    delta_pose = init_delta_pose_stamped()

    rospy.sleep(3)

    init_ring = True
    init_panda = True
    dataset = []
    current_pose_stamped = None

    for i in range(1):
        while not rospy.is_shutdown():
            if init_panda:
                pub_joint_controller.publish(start_joint_states)
                init_panda = False
                wait_move(start_joint_states, move_group)
                current_pose_stamped = move_group.get_current_pose()
                continue
            if init_ring:
                ring_point = get_valid_ring_position()
                ring_pose = Pose()
                ring_pose.position = ring_point
                pub_ring.publish(ring_pose)
                # delta between vrep and rviz on x of 0.5!!
                ring_pose.position.x += 0.5
                init_ring = False
                rospy.sleep(1)
                continue
            if get_coeff(current_pose_stamped, ring_point) < GRASP_DISTANCE:
                rospy.sleep(3)
                init_ring = True
                init_panda = True
                break

            new_point = get_master_policy(current_pose_stamped, ring_point)
            delta_pose.pose.position = new_point
            current_pose_stamped.pose.position.x = current_pose_stamped.pose.position.x + new_point.x
            current_pose_stamped.pose.position.y = current_pose_stamped.pose.position.y + new_point.y
            current_pose_stamped.pose.position.z = current_pose_stamped.pose.position.z + new_point.z

            # append images and points to dataset
            dataset.append((LAST_IMAGE, new_point))

            # do delta movement
            pub_delta_controller.publish(delta_pose)

            # wait until movement is done
            wait_move(current_pose_stamped, move_group)

    # save dataset to file
    with open("dataset.pkl", mode="wb") as fd:
        pickle.dump(dataset, fd)


if __name__ == '__main__':
    main()
