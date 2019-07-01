#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray
from rl_environment import Env
from turtlebot2i_safety.msg import SafetyRisk


if __name__ == '__main__':
    try:
        rospy.init_node('training_rl_py')
        env = Env()
        print("start testing")
        while True:
            next_state, reward, done = env.step(8)
            #raw_input("Press Enter to continue...")

    except rospy.ROSInterruptException:
        env.vrep_control.shutdown()
        #vrep.simxFinish(clientID)
        rospy.loginfo("Training finished.")
        a = 2.0/0.0 #create an error to terminate program :/