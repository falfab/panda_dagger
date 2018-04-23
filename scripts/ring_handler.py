import random
import math
from datetime import datetime
from geometry_msgs.msg import Pose, Point

from config_handler import ConfigHandler


class RingHandler(ConfigHandler):

    def __init__(self):
        super(RingHandler, self).__init__()
        self.ring_coordinate = Point()
        random.seed(datetime.now())

    def set_random_valid_pose(self):
        min_x = self.conf.getfloat('Ring', 'MinX')
        max_x = self.conf.getfloat('Ring', 'MaxX')
        min_y = self.conf.getfloat('Ring', 'MinY')
        max_y = self.conf.getfloat('Ring', 'MaxY')
        self.ring_coordinate = Point(
            min_x + (random.random() * (max_x - max_x)),
            min_y + (random.random() * (max_y - min_y)),
            self.conf.getfloat('Ring', 'Height')
        )

    def get_ring_pose(self):
        pose = Pose()
        pose.position = self.ring_coordinate
        return pose

    def get_grasp_point(self):
        """
        Returns grasp point given the circle center position
        (geometry_msgs.msg.Point)
        """
        ring_point = self.ring_coordinate
        return Point(
            self.ring_coordinate.x,
            (ring_point.y - self.conf.getfloat('Ring', 'Radius')) if (self.ring_coordinate.y > 0)
            else (self.ring_coordinate.y + self.conf.getfloat('Ring', 'Radius')),
            self.conf.getfloat('Goal', 'GoalHeight')
        )
        
    def is_ring_visible(self, current_pose):
        ring_radius = self.conf.getfloat('Ring','Radius') 
    
        # delta = lenght/2 of vision's square sides
        # coordinates of the visible square on surface
        robot_z = current_pose.pose.position.z - ring_radius # TODO add offset!
        
        delta = robot_z * math.sqrt(3)/3
        min_x = current_pose.pose.position.x - delta
        max_x = current_pose.pose.position.x + delta        
        min_y = current_pose.pose.position.y - delta
        max_y = current_pose.pose.position.y + delta        
        
        # for edges point for ring
        p1x = self.ring_coordinate.x - ring_radius
        p1y = self.ring_coordinate.y
        p2x = self.ring_coordinate.x + ring_radius
        p2y = self.ring_coordinate.y
        p3x = self.ring_coordinate.x
        p3y = self.ring_coordinate.y - ring_radius
        p4x = self.ring_coordinate.x
        p4y = self.ring_coordinate.y + ring_radius
        
        return (p1x > min_x and p1x < max_x and p1y > min_y and p1y < max_y) or \
               (p2x > min_x and p2x < max_x and p2y > min_y and p2y < max_y) or \
               (p3x > min_x and p3x < max_x and p3y > min_y and p3y < max_y) or \
               (p4x > min_x and p4x < max_x and p4y > min_y and p4y < max_y)



