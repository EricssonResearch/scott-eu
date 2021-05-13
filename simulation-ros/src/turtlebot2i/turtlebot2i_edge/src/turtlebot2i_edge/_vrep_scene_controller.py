import vrep
import rospy
import time
from turtlebot2i_scene_graph import VrepClient


class VrepSceneController(VrepClient):
    def __init__(self):
        super(VrepSceneController, self).__init__()

    def reset(self):
        if self.is_running():
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
            self._wait_stopped_simulation()
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
            self.wait_running_simulation()
            rospy.loginfo('Simulation reset')
        else:       # another node is already resetting, just wait
            self.wait_running_simulation()

    def _wait_stopped_simulation(self):
        while self.is_running():
            time.sleep(1)

    def wait_running_simulation(self):
        while not self.is_running():
            time.sleep(1)

    def is_running(self):
        _, handles = vrep.simxGetObjects(self.clientID, vrep.sim_object_visionsensor_type, vrep.simx_opmode_blocking)
        return len(handles) > 0
