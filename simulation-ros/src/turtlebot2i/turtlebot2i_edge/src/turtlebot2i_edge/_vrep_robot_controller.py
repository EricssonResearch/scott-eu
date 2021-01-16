import vrep
from turtlebot2i_scene_graph import VrepClient


class VrepRobotController(VrepClient):
    def __init__(self):
        super(VrepRobotController, self).__init__()
        self._handle = None
        self._position = None

    def load_robot(self, robot):
        return_code, self._handle = vrep.simxGetObjectHandle(self.clientID, robot, vrep.simx_opmode_blocking)
        if return_code != vrep.simx_return_ok:
            raise ValueError('%s not found' % robot)
        _, self._position = vrep.simxGetObjectPosition(self.clientID, self._handle, -1, vrep.simx_opmode_blocking)

    def reset_robot_position(self):
        if self._handle is None:
            raise Exception('Robot not loaded')
        vrep.simxSetObjectPosition(self.clientID, self._handle, -1, self._position, vrep.simx_opmode_blocking)
