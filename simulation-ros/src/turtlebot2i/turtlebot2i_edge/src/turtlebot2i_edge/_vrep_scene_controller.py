import vrep
from turtlebot2i_scene_graph import VrepClient


class VrepSceneController(VrepClient):
    def __init__(self):
        super(VrepSceneController, self).__init__()

    def reset(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
