import vrep
from turtlebot2i_scene_graph import VrepClient


class VrepRobotController(VrepClient):
    def __init__(self, robot):
        super(VrepRobotController, self).__init__()

        return_code, self._robot = vrep.simxGetObjectHandle(self.clientID, robot, vrep.simx_opmode_oneshot_wait)
        if return_code != 0:
            raise ValueError('%s not found' % robot)

    def move_robot(self, pose):
        vrep.simxSetObjectPosition(self.clientID, self._robot, -1, pose.position, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectQuaternion(self.clientID, self._robot, -1, pose.orientation, vrep.simx_opmode_oneshot_wait)
