"""
RobotBase
@author Ricardo Souza
@date 21-06-2016
Description: Defines abstract class RobotBase
"""

from abc import *

class RobotBase(ABC):

    @abstractmethod
    def move_to(self, robot, destination):
        """Send robot to destination"""
        pass

    @abstractmethod
    def move_done(self, robot):
        """Check if robot is moving"""
        pass

    @abstractmethod
    def pick_up(self, object_):
        """Send pickup command to robots"""
        pass

    @abstractmethod
    def drop_at(self, object_, place):
        """Drops object at specifig place"""
        pass

    @abstractmethod
    def get_cargo(self, robot_ID):
        """Returns robot's current cargo"""
        pass

    @abstractmethod
    def get_pose(self, robot_ID):
        """Returns robot's current pose (x, y, th)"""
        pass
