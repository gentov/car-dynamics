#!/usr/bin/env python3
import math

import rospy
import sys

from PyQt5.uic.properties import QtGui
from std_msgs.msg import Float64
import numpy as np
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QSizePolicy
from PyQt5.QtCore import Qt
from carpackage.msg import TrajectoryMSG
from carpackage.srv import VandWService, VandWServiceResponse
from PyQt5.QtCore import QThread, pyqtSignal
designerFile = "/home/nick/catkin_ws/src/501Project/carpackage/src/car-dynamics/CarApp.ui"
class CarApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(CarApp, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.KeyboardMessage = rospy.Publisher('/CarKeyboard', Float64, queue_size=1)
        self.RobotMode = rospy.Publisher('/CarMode', Float64, queue_size=1)
        self.VandWService = rospy.wait_for_service("VandW")

        ## Custom Trajectory message: a message that has a list for X, Y, Theta, and Time
        self.Trajectory = TrajectoryMSG()
        self.TrajectoryMessagePublisher = rospy.Publisher('/Trajectory', TrajectoryMSG, queue_size=1)
        self.Mode = "Tele-Op"
        self.ModeCombo.addItem("Tele-Op")
        self.ModeCombo.addItem("Test Mode")
        self.ModeCombo.addItem("I/O Controller")
        self.ModeCombo.activated.connect(self.ChangeMode)
        self.SendVandWBTN.clicked.connect(self.SendWandVCallback)
        self.AddArcBTN.clicked.connect(self.addArcCallback)
        self.AddLineBTN.clicked.connect(self.addLineCallback)
        self.SendSquareBTN.clicked.connect(self.generateSquareTrajectory)
        self.SendCustomTrajectoryBTN.clicked.connect(self.generateCustomTrajectory)

        self.TrajectoryDesignerList = []
        self.TrajListIndex = 0

    def generateSquareTrajectory(self):
        # We can start by making a square with N*4 waypoints, where N is the number of 
        # Waypoints per side, we will say 10 for now
        wayPointsPerSide = 10
        waypointsTotal = wayPointsPerSide*4
        # Different Headings of the robot
        thetaChoices = [0, 90, 180, 270]
        x = [0]
        y = [0]
        time = [0]
        theta = [0]
        side = 0
        # This is essentially the increment as we move along the square 
        speed = 5
        timeStep = 1

        previousX = 0
        previousY = 0
        #Create a trajectory for the square
        for i in range(waypointsTotal):
            # Determine which is the proper heading depending on the side we're on
            thetaIndex = i/wayPointsPerSide
            theta.append(thetaChoices[thetaIndex])
            #Increment time step. Initially starts at 0
            time.append(time[i] + timeStep)
            # Initially, side will be by default, set to 1 (0 % 10 == 0)
            if(i % wayPointsPerSide == 0):
                side = side + 1
            # If we're on the first side, only increment x (up)
            if (side == 1):
                # the increment looks at the previous X and adds to it
                x.append(previousX + speed)
                y.append(previousY)
            # If we're on the second side, only increment Y (up)
            elif (side == 2):
                x.append(previousX)
                y.append(previousY + speed)
            # If we're on the third side, only increment x (down)
            elif (side == 3):
                x.append(previousX - speed)
                y.append(previousY)
            # If we're on the fourth side, only increment Y (down)
            elif (side == 4):
                x.append(previousX)
                y.append(previousY - speed)
            
            #update previous X and Y
            previousX = x[-1]
            previousY = y[-1]

        # Populate X,Y,Theta,Time with message type arrays
        self.Trajectory.X = x
        self.Trajectory.Y = y
        self.Trajectory.Theta = theta
        self.Trajectory.time = time

        self.TrajectoryMessagePublisher.publish(self.Trajectory)
        # A button in a Qt window will have this method as its callback
        # This will publish the trajectory message, and the InputOutController will subscribe to this message type

    def generateCustomTrajectory(self):
        # Iterate through TrajDesignerList
        # Calculate speed based on degrees and radius (arclength) and time
        # Initialize everything at XY Theta is 0
        # Take time, break up each movement, arc or line into time into half second segments
        # Calculate steering angle from radius
            #Use equations of motion to figure out what the desired X,Y,Theta is
        # If line, straighten the wheels (set psi to 0)
            # Use equations of motion to figure out desired X, Y, Theta is
        # Populate trajectory message type
        pass
        

    #overriding keyPressEvent from Qt library
    def keyPressEvent(self, event):
        if self.Mode == "Tele-Op":
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
    def ChangeMode(self):
        self.Mode = self.ModeCombo.text()
        if(self.Mode == "Tele-Op"):
            self.RobotMode.publish(0)
        elif(self.Mode == "Test Mode"):
            self.RobotMode.publish(1)
        elif(self.Mode =="I/O Controller"):
            self.RobotMode.publish(2)

    def SendWandVCallback(self):
        if self.Mode =="Test Mode":
            try:
                V = int(self.SetV.text())
            except:
                V = 0
                self.SetV.setText("0")
            try:
                W = int(self.SetW.text())
            except:
                W =0
                self.SetW.setText("0")
            vals = self.limiter(V, W)
            self.SetV.setText(vals[0].toString())
            self.SetW.setText(vals[1].toString())
            sendVandW = rospy.ServiceProxy('VandW',VandWService)
            response = sendVandW(vals[0], vals[1])

    def limiter(self, V, W):
        # This takes in the commanded V and W from the controller, and limits the values to
        # values that the car can actually do
        # Use W to calculate psi, then limit psi
        # Use new psi to calculate W
        # Limit V to feasible value and send it
        carLength = .212
        carWidth  = .224
        limitedV = 0
        velocityLimit  = 100
        if(V > velocityLimit):
            limitedV = velocityLimit
        elif(V < -velocityLimit):
            limitedV = -velocityLimit
        psiTemp = math.atan(W*carLength/V)
        if(psiTemp > 45):
            psiTemp = 45
        if(psiTemp < -45):
            psiTemp = -45
        limitedW = math.tan(psiTemp)/carLength*V
    
        return(limitedV,limitedW)

    def addArcCallback(self):
        try:
            Radius = float(self.RadiusEntry.text())
        except:
            Radius = 1
            self.RadiusEntry.setText("1")

        try:
            Degrees = float(self.DegreesEntry.text())
        except:
            Degrees = 90.0
            self.DegreesEntry.setText("90.0")

        try:
            time = float(self.TimeArcEntry.text())
            if time<=0:
                time = 1
                self.TimeArcEntry.setText("1")
        except:
             time = 1
             self.TimeArcEntry.setText("1")

        NewItem = ("Arc", Radius, Degrees, time)
        self.TrajectoryDesignerList.append(NewItem)
        self.TrajList.insertItem(self.TrajListIndex, " Arc "+str(Radius)+" "+str(Degrees)+" "+str(time))
        self.TrajListIndex+=1

    def addLineCallback(self):
         try:
            Distance = float(self.DistanceEntry.text())
         except:
            Distance = 1
            self.DistanceEntry.setText("1")

         try:
            time = float(self.TimeLineEntry.text())
            if time<=0:
                time = 1
                self.TimeLineEntry.setText("1")
         except:
             time = 1
             self.TimeLineEntry.setText("1")
         NewItem = ("Line", Distance, time) 
         self.TrajectoryDesignerList.append(NewItem)
         self.TrajList.insertItem(self.TrajListIndex, " Line "+str(Distance)+" "+str(time))
         self.TrajListIndex+=1



if __name__ == '__main__':  
    rospy.init_node('Car_Control')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    window = CarApp()
    window.show()
    sys.exit(app.exec_())