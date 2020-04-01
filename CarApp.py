#!/usr/bin/env python3
import rospy
import sys

from PyQt5.uic.properties import QtGui
from std_msgs.msg import Float64
import numpy as np
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QSizePolicy
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QThread, pyqtSignal
designerFile = "/home/nick/catkin_ws/src/501Project/carpackage/src/car-dynamics/CarApp.ui"
class CarApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(CarApp, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.KeyboardMessage = rospy.Publisher('/CarKeyboard', Float64, queue_size=1)
        self.RobotMode = rospy.Publisher('/CarMode', Float64, queue_size=1)

        ## Custom Trajectory message: a list containing x,y,theta
        self.TrajectoryMessage = rospy.Publisher('/Trajectory', TrajectoryMSG, queue_size=1)
        
    def generateTrajectory():
        # A button in a Qt window will have this method as its callback
        # This will publish the trajectory message, and the InputOutController will subscribe to this message type
        pass

        #overriding from Qt library
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Up:
            print("Pressed Up")
            self.KeyboardMessage.publish(0)
        elif event.key() == Qt.Key_Down:
            print("Pressed Down")
            self.KeyboardMessage.publish(1)
        elif event.key() == Qt.Key_Left:
            print("Pressed Left")
            self.KeyboardMessage.publish(2)
        elif event.key() == Qt.Key_Right:
            print("Pressed Right")
            self.KeyboardMessage.publish(3)
        elif event.key() == Qt.Key_Space:
            self.KeyboardMessage.publish(4)
        elif event.key() == Qt.Key_B:
            self.KeyboardMessage.publish(5)
        elif event.key() == Qt.Key_S:
            self.KeyboardMessage.publish(6)

if __name__ == '__main__':
    rospy.init_node('Car_Control')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    window = CarApp()
    window.show()
    sys.exit(app.exec_())