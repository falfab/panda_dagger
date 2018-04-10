import math
import moveit_commander
import rospy
from config_handler import ConfigHandler
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import JointState


class MoveItHandler(ConfigHandler):

    def __init__(self, move_group_name='panda_arm'):
        super(MoveItHandler, self).__init__()
        self.move_group = moveit_commander.MoveGroupCommander(move_group_name)
        # init ready position
        self.target_pose = PoseStamped()
        self.target_joint_states = JointState()
        self.target_joint_states.position = [
            0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        # init delta pose
        self.delta_pose = PoseStamped()
        self.delta_pose.pose.position = Point(0., 0., 0.)
        self.delta_pose.pose.orientation.x = 0.923955
        self.delta_pose.pose.orientation.y = -0.382501
        self.delta_pose.pose.orientation.z = -0.000045
        self.delta_pose.pose.orientation.w = 0.000024
        self.current_pose = None
        self.current_joint_states = None
        self.grasp_point = Point()

    def update_current_pose(self):

        self.current_pose = self.move_group.get_current_pose()

    def update_current_joint_values(self):

        self.current_joint_states = self.move_group.get_current_joint_values()

    def get_step_size(self, ring_coodrdinate):

        self.update_current_pose()
        robot_point = self.current_pose.pose.position
        d_x = robot_point.x - ring_coodrdinate.x
        d_y = robot_point.y - ring_coodrdinate.y
        d_z = robot_point.z - ring_coodrdinate.z
        d_xy = math.sqrt(d_x**2 + d_y**2)
        distance = math.sqrt(d_xy**2 + d_z**2)

        return distance * self.conf.getfloat('MoveIt', 'StepRatio')

    def wait(self, target):

        if isinstance(target, PoseStamped) or isinstance(target, Pose):
            self.update_current_pose()
            while not self.equal_poses(self.current_pose.pose, target.pose):
                self.update_current_pose()
                rospy.sleep(0.1)
        elif isinstance(target, JointState):
            self.update_current_joint_values()
            while not self.equal_joint_states(self.current_joint_states, target.position):
                self.update_current_joint_values()
                rospy.sleep(0.1)
        else:
            raise ValueError(
                'target object passed to wait_move() is neither a pose or jointstates')

    def equal_poses(self, pose1, pose2):
        val = abs(pose1.position.x - pose2.position.x) < self.conf.getfloat('MoveIt', 'PositionTolerance') and \
            abs(pose1.position.y - pose2.position.y) < self.conf.getfloat('MoveIt', 'PositionTolerance') and \
            abs(pose1.position.z -
                pose2.position.z) < self.conf.getfloat('MoveIt', 'PositionTolerance')

        return val

    def equal_joint_states(self, joint_array1, joint_array2):
        if len(joint_array1) != len(joint_array2):
            return False
        for (index, joint1) in enumerate(joint_array1):
            if abs(joint1 - joint_array2[index]) > self.conf.getfloat('MoveIt', 'PositionTolerance'):
                return False
        return True

    def update_target_pose(self):
        self.target_pose.pose.position.x = self.current_pose.pose.position.x + \
            self.delta_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y + \
            self.delta_pose.pose.position.y
        self.target_pose.pose.position.z = self.current_pose.pose.position.z + \
            self.delta_pose.pose.position.z

    def compute_master_policy(self, ring_handler):
        """
        Returns master policy for delta controller
        (a unit vector pointing the ring as a triple)
        """
        ss = self.get_step_size(ring_handler.ring_coordinate)

        grasp_point = ring_handler.get_grasp_point()
        d_x = grasp_point.x - self.current_pose.pose.position.x
        d_y = grasp_point.y - self.current_pose.pose.position.y
        d_z = grasp_point.z - self.current_pose.pose.position.z

        if abs(d_x) > self.conf.getfloat('MoveIt', 'PositionTolerance'):
            alpha_yx = math.atan2(d_y, d_x)  # b/a
            alpha_zx = math.atan2(d_z, d_x)
            self.delta_pose.pose.position = Point(
                ss * math.cos(alpha_yx), ss * math.sin(alpha_yx), ss * math.sin(alpha_zx))
        elif abs(d_y) > self.conf.getfloat('MoveIt', 'PositionTolerance'):
            alpha_zy = math.atan2(d_z, d_y)
            self.delta_pose.pose.position = Point(
                0., ss * math.cos(alpha_zy), ss * math.sin(alpha_zy))
        elif abs(d_z) > self.conf.getfloat('MoveIt', 'ZAxisTolerance'):
            self.delta_pose.pose.position = Point(0., 0., ss * -1.)
        else:
            self.delta_pose.pose.position = Point(0., 0., 0.)
