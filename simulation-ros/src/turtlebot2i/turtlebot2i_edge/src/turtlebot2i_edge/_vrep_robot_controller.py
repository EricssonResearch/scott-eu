import os
import vrep
from turtlebot2i_scene_graph import VrepClient


class VrepRobotController(VrepClient):
    def __init__(self, models_path):
        super(VrepRobotController, self).__init__()
        self._robot = None      # handle
        self._model_path = os.path.join(models_path, 'turtlebot2i.ttm')

    def load_robot(self, robot):
        return_code, self._robot = vrep.simxGetObjectHandle(self.clientID, robot, vrep.simx_opmode_blocking)
        if return_code != vrep.simx_return_ok:
            raise ValueError('%s not found' % robot)

    def reset(self):
        if self._robot is not None:
            vrep.simxRemoveModel(self.clientID, self._robot, vrep.simx_opmode_blocking)
        _, self._robot = vrep.simxLoadModel(self.clientID, self._model_path, 0, vrep.simx_opmode_blocking)
