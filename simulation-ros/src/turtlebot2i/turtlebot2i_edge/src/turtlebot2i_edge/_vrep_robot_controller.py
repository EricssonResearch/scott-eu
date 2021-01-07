import vrep
from turtlebot2i_scene_graph import VrepClient


class VrepRobotController(VrepClient):
    def __init__(self):
        super(VrepRobotController, self).__init__()
        self._robot = None      # handle

    def load_robot(self, robot):
        return_code, self._robot = vrep.simxGetObjectHandle(self.clientID, robot, vrep.simx_opmode_oneshot_wait)
        if return_code != 0:
            raise ValueError('%s not found' % robot)

    def move_robot(self, pose):
        position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        vrep.simxSetObjectPosition(self.clientID, self._robot, -1, position, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectQuaternion(self.clientID, self._robot, -1, orientation, vrep.simx_opmode_oneshot_wait)
