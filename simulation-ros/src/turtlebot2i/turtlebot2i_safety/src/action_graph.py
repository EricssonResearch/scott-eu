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

import rospy
import sys
from std_msgs.msg import Float32MultiArray, Float32
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QProgressBar
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtCore import QThread
from PyQt5.QtCore import QWaitCondition
from PyQt5.QtCore import QMutex
from PyQt5.QtCore import pyqtSignal
from PyQt5 import QtGui, QtCore
from std_msgs.msg import Float64
from turtlebot2i_safety.msg import VelocityScale

class Thread(QThread):
    change_value1 = pyqtSignal(int)
    change_value2 = pyqtSignal(int)
    change_value6 = pyqtSignal(str)
    change_value7 = pyqtSignal(str)
    risk_value    = pyqtSignal(str)
    status_value  = pyqtSignal(str)
    l_scale_value = pyqtSignal(int)
    r_scale_value = pyqtSignal(int)

    def __init__(self):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self.mutex = QMutex()
        self.cnt = 0
        self._status = True
        self.sub = rospy.Subscriber('/turtlebot2i/safety/risk_val', Float64, self.get_risk_value)
        self.vel_scale_sub = rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, self.speed_scale_callback)
        #self.l_scale = 1.0
        #self.r_scale = 1.0

    def get_risk_value(self, array):
        data = []
        risk_val = array.data
        self.risk_value.emit(str(round(array.data, 2)))
        if risk_val > 3.0:
            self.status_value.emit('Critical')
        elif risk_val > 2.0:
            self.status_value.emit('Warning')
        else:
            self.status_value.emit('Safe')
        '''
        self.array = array
        self.array.data = list(self.array.data)
        for i in self.array.data[:-2]:
            data.append(float(i * 100))

        self.change_value1.emit(0)
        self.change_value2.emit(0)

        self.change_value1.emit(100*self.array.data[0])
        self.change_value2.emit(100*self.array.data[1])

        self.change_value6.emit(str(round(self.array.data[-2], 2)))
        self.change_value7.emit(str(round(self.array.data[-1], 2)))
        '''
    def speed_scale_callback(self, data):
        self.l_scale_value.emit(100*data.left_vel_scale/1.2)
        self.r_scale_value.emit(100*data.right_vel_scale/1.2)

class Form(QWidget):
    def __init__(self):
        QWidget.__init__(self, flags=Qt.Widget)

        self.pgsb1 = QProgressBar()
        self.pgsb1.setOrientation(QtCore.Qt.Vertical)
        self.pgsb1.setValue(0)
        self.pgsb1.setRange(0, 100)
        self.pgsb1.setFormat(" ")

        self.pgsb2 = QProgressBar()
        self.pgsb2.setValue(0)
        self.pgsb2.setRange(0, 100)
        self.pgsb2.setOrientation(QtCore.Qt.Vertical)
        self.pgsb2.setFormat(" ")

        #self.label_front = QLabel("Front", self)
        self.label_left = QLabel("Left", self)
        self.label_right = QLabel("Right", self)

        self.total_reward_Label = QLabel(self)
        self.total_reward_Label.setText('Safety status')

        self.total_reward = QLineEdit("  ", self)
        self.total_reward.setDisabled(True)

        self.reward_Label = QLabel(self)
        self.reward_Label.setText('Risk Value')

        self.reward = QLineEdit("  ", self)
        self.reward.setDisabled(True)

        self.th = Thread()
        self.init_widget()
        self.th.start()

    def init_widget(self):
        super(Form, self).__init__()
        self.setWindowTitle("Action State")
        self.setGeometry(0, 0, 200, 100)
        form_lbx = QGridLayout()

        self.th.l_scale_value.connect(self.pgsb1.setValue)
        self.th.r_scale_value.connect(self.pgsb2.setValue)
        self.th.status_value.connect(self.total_reward.setText)
        self.th.risk_value.connect(self.reward.setText)

        form_lbx.addWidget(self.pgsb1, 0, 4, 4, 1)
        form_lbx.addWidget(self.pgsb2, 0, 6, 4, 1)

        self.total_reward.setFixedWidth(100)
        self.reward.setFixedWidth(100)

        #form_lbx.addWidget(self.label_front, 4, 6)
        form_lbx.addWidget(self.label_left, 4, 4)
        form_lbx.addWidget(self.label_right, 4, 6)
        form_lbx.addWidget(self.total_reward_Label, 0, 0)
        form_lbx.addWidget(self.total_reward, 1, 0)
        form_lbx.addWidget(self.reward_Label, 2, 0)
        form_lbx.addWidget(self.reward, 3, 0)
        self.setLayout(form_lbx)

if __name__ == "__main__":
    rospy.init_node('progress')
    app = QApplication(sys.argv)
    form = Form()
    form.show()
    exit(app.exec_())
