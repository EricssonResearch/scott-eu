import vrep
import rospy
import time
from turtlebot2i_scene_graph import VrepClient


class VrepSceneController(VrepClient):
    def __init__(self):
        super(VrepSceneController, self).__init__()

    def reset(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        self._wait_simulation_stopped()
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        self._wait_simulation_started()
        rospy.loginfo('Simulation reset')

    def _wait_simulation_stopped(self):
        handles = None
        mode = vrep.simx_opmode_blocking
        while handles is None or len(handles) > 0:
            _, handles = vrep.simxGetObjects(self.clientID, vrep.sim_object_visionsensor_type, mode)
            time.sleep(1)

    def _wait_simulation_started(self):
        handles = []
        mode = vrep.simx_opmode_blocking
        while len(handles) == 0:
            _, handles = vrep.simxGetObjects(self.clientID, vrep.sim_object_visionsensor_type, mode)
            time.sleep(1)
