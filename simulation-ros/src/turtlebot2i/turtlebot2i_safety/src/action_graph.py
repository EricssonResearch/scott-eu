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
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from sensor_msgs.msg import Joy

class Thread(QThread):
    change_value1 = pyqtSignal(int)
    change_value2 = pyqtSignal(int)
    change_value6 = pyqtSignal(str)
    change_value7 = pyqtSignal(str)
    risk_value    = pyqtSignal(str)
    status_value  = pyqtSignal(str)
    l_scale_value = pyqtSignal(int)
    r_scale_value = pyqtSignal(int)
    angular_speed_value = pyqtSignal(int)
    linear_speed_value = pyqtSignal(int)

    def __init__(self):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self.mutex = QMutex()
        self.cnt = 0
        self._status = True
        self.sub = rospy.Subscriber('/turtlebot2i/safety/risk_val', Float64, self.get_risk_value)
        self.vel_scale_sub = rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, self.speed_scale_callback)
        self.nav_sub = rospy.Subscriber('/turtlebot2i/navigation/velocity', Twist, self.speed_callback) 
        self.joystick_mode = False
        rospy.Subscriber('/turtlebot2i/commands/led1', Led, self.led_callback)

        #self.nav_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback)
        #self.joystick_mode = True

        
    def led_callback(self, data):
        self.joystick_mode = False
        if data.value == Led.BLACK:
            self.nav_sub.unregister()
            #print("Unsubscribe to any navigation module")
        elif data.value == Led.GREEN:
            self.nav_sub.unregister()
            self.joystick_mode = True
            #nav_sub = rospy.Subscriber('/turtlebot2i/keyop/velocity', Twist, speed_callback)
            self.nav_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback)
            #print("Subscribed to keyop")
        elif data.value == Led.ORANGE or data.value == Led.RED:
            self.nav_sub.unregister()
            self.nav_sub = rospy.Subscriber('/turtlebot2i/navigation/velocity', Twist, self.speed_callback)
            #print("Subscribed to move_base")

    def joystick_callback(self, joystick_data):
        #print('something')
        if joystick_data.axes[6] == 0.0 and joystick_data.axes[7] == 0.0: 
            #speed.linear.x  = 0.3 * joystick_data.axes[1] #max 1.5
            #speed.angular.z = 1.0 * joystick_data.axes[0] #max 6.6
            if joystick_data.axes[1] < 0.0:
                self.linear_speed_value.emit(100*min(0.1, -joystick_data.axes[1]))
            else:
                self.linear_speed_value.emit(100*joystick_data.axes[1])
            self.angular_speed_value.emit(100*joystick_data.axes[0])
        else:
            #speed.linear.x  = 0.3 * joystick_data.axes[7] #max 1.5
            #speed.angular.z = 0.4 * joystick_data.axes[6] #max 6.6
            self.linear_speed_value.emit(100*joystick_data.axes[7])
            self.angular_speed_value.emit(100*joystick_data.axes[6])

    def speed_callback(self, data):
        #max_vel_x: 0.3 
        #max_vel_theta: 0.5
        self.linear_speed_value.emit(100*data.linear.x/0.31)
        self.angular_speed_value.emit(100*data.angular.z/0.51)



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

        self.pgsb3 = QProgressBar()
        self.pgsb3.setValue(0)
        self.pgsb3.setRange(0, 100)
        self.pgsb3.setOrientation(QtCore.Qt.Horizontal)
        self.pgsb3.setFormat(" ")

        self.pgsb4 = QScrollBar()
        self.pgsb4.setValue(0)
        self.pgsb4.setRange(-100, 100)
        self.pgsb4.setOrientation(QtCore.Qt.Horizontal)
        #self.pgsb3.setFormat(" ")

        #self.label_front = QLabel("Front", self)
        self.label_left = QLabel("Left", self)
        self.label_right = QLabel("Right", self)
        self.label_linear_speed = QLabel("Linear speed", self)
        self.label_angular_speed = QLabel("Angular speed", self)

        self.safety_status_label = QLabel(self)
        self.safety_status_label.setText('Safety status')

        self.safety_status = QLineEdit("  ", self)
        self.safety_status.setDisabled(True)

        self.risk_label = QLabel(self)
        self.risk_label.setText('Risk Value')

        self.risk = QLineEdit("  ", self)
        self.risk.setDisabled(True)

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
        self.th.status_value.connect(self.safety_status.setText)
        self.th.risk_value.connect(self.risk.setText)

        self.th.linear_speed_value.connect(self.pgsb3.setValue)
        self.th.angular_speed_value.connect(self.pgsb4.setValue)

        form_lbx.addWidget(self.pgsb1, 40, 4, 4, 1)
        form_lbx.addWidget(self.pgsb2, 40, 6, 4, 1)
        form_lbx.addWidget(self.pgsb3, 5, 0, 10, 10)
        form_lbx.addWidget(self.pgsb4, 25, 0, 10, 10)

        self.safety_status.setFixedWidth(100)
        self.risk.setFixedWidth(100)

        #form_lbx.addWidget(self.label_front, 4, 6)
        form_lbx.addWidget(self.label_linear_speed, 0, 3)
        form_lbx.addWidget(self.label_angular_speed, 20, 3)
        form_lbx.addWidget(self.label_left, 44, 4)
        form_lbx.addWidget(self.label_right, 44, 6)
        form_lbx.addWidget(self.safety_status_label, 40, 0)
        form_lbx.addWidget(self.safety_status, 41, 0)
        form_lbx.addWidget(self.risk_label, 42, 0)
        form_lbx.addWidget(self.risk, 43, 0)
        self.setLayout(form_lbx)

if __name__ == "__main__":
    rospy.init_node('progress')
    app = QApplication(sys.argv)
    form = Form()
    form.show()
    exit(app.exec_())
