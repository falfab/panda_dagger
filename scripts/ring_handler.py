import random
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
