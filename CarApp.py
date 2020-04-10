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
import matplotlib.pyplot as plt 

designerFile = "/home/nick/catkin_ws/src/501Project/carpackage/src/car-dynamics/CarApp.ui"

class CarApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(CarApp, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.KeyboardMessage = rospy.Publisher('/CarKeyboard', Float64, queue_size=1)
        self.RobotMode = rospy.Publisher('/CarMode', Float64, queue_size=1)
        self.VandWService = rospy.wait_for_service("VandW")

        # Custom Trajectory message: a message that has a list for X, Y, Theta, and Time
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
        self.ForwardBTN.clicked.connect(self.forwardCallback)
        self.BackwardBTN.clicked.connect(self.backwardCallback)
        self.LeftBTN.clicked.connect(self.leftCallback)
        self.RightBTN.clicked.connect(self.rightCallback)
        self.BrakeBTN.clicked.connect(self.brakeCallback)
        self.StraightBTN.clicked.connect(self.straightCallback)

        self.TrajectoryDesignerList = []
        self.TrajListIndex = 0

        self.ForwardBTN.setEnabled(True)
        self.BackwardBTN.setEnabled(True)
        self.LeftBTN.setEnabled(True)
        self.RightBTN.setEnabled(True)
        self.BrakeBTN.setEnabled(True)
        self.StraightBTN.setEnabled(True)
        self.SendVandWBTN.setEnabled(False)
        self.AddArcBTN.setEnabled(False)
        self.AddLineBTN.setEnabled(False)
        self.SendSquareBTN.setEnabled(False)
        self.SendCustomTrajectoryBTN.setEnabled(False)


    def generateSquareTrajectory(self):
        # We can start by making a square with N*4 waypoints, where N is the number of 
        # Waypoints per side, we will say 10 for now
        wayPointsPerSide = 100
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
            thetaIndex = int(i/wayPointsPerSide)
            theta.append(thetaChoices[thetaIndex])
            #Increment time step. Initially starts at 0
            time.append(time[i] + timeStep)
            # Initially, side will be by default, set to 1 (0 % 10 == 0)
            if(i % wayPointsPerSide == 0):
                side = side + 1
            # If we're on the first side, only increment x (up)
            if (side == 1):
                # the increment looks at the previous X and adds to it
                x.append(previousX + (speed * timeStep))
                y.append(previousY)
            # If we're on the second side, only increment Y (up)
            elif (side == 2):
                x.append(previousX)
                y.append(previousY + (speed* timeStep))
            # If we're on the third side, only increment x (down)
            elif (side == 3):
                x.append(previousX - (speed * timeStep))
                y.append(previousY)
            # If we're on the fourth side, only increment Y (down)
            elif (side == 4):
                x.append(previousX)
                y.append(previousY - (speed * timeStep))
            
            
            #update previous X and Y
            previousX = x[-1]
            previousY = y[-1]

        # Populate X,Y,Theta,Time with message type arrays
        self.Trajectory.X = x
        self.Trajectory.Y = y
        self.Trajectory.Theta = theta
        self.Trajectory.time = time
        # plt.xlim(-50, 550)
        # plt.ylim(-50, 550)
        # plt.plot(x,y)
        # plt.show()

        self.TrajectoryMessagePublisher.publish(self.Trajectory)
        # A button in a Qt window will have this method as its callback
        # This will publish the trajectory message, and the InputOutController will subscribe to this message type

    def generateCustomTrajectory(self):
        print("Here")
        # Some useful Car Parameters:
        carWidth_mm = 224
        carLength_mm = 212
        # Initial X, Y, Time, Theta
        # Initialize everything at XY Theta is 0
        x = [0]
        y = [0]
        time = [0]
        theta = [0]
        #@TODO: Change this time step. It just looks nicer when the resolution is finer
        timeStep = .05
        #Speed to increment 
        speed = 0
        previousX = 0
        previousY = 0

        # Iterate through the components of the desired trajectory
        for trajectory in self.TrajectoryDesignerList:
            # Determine the type of component:
            if(trajectory[0] == "Line"):
                # Calculate the speed based on distance and time
                distance = trajectory[1]
                totalTime = trajectory[2]
                speed = distance/totalTime
                
                # We will have totalTime/timeStep waypoints.
                # If we have 10 seconds, and a timestep of 1 second, we should 
                # have 10 waypoints.
                for i in range(int(totalTime/timeStep)):
                    # If line, we keep the theta the same as before, no change in theta
                    # theta[-1] is a nifty way of just grabbing the last element in the array
                    # Use equations of motion to figure out desired X, Y, Theta is
                    x.append(previousX + speed*math.cos(theta[-1]) * timeStep)
                    y.append(previousY + speed*math.sin(theta[-1]) * timeStep)
                    theta.append(theta[-1])

                    #Update where we are right now
                    previousX = x[-1]
                    previousY = y[-1]
                    time.append(time[-1] + timeStep)
            else:
                # Calculate speed based on degrees and radius (arclength) and time
                radius = trajectory[1]
                degrees = trajectory[2]
                totalTime = trajectory[3]
                print("Radius", radius)
                print("Degrees", degrees)
                print("totalTime", totalTime)
                # arclength = 2*pi*r*(theta/360)
                arclength = 2*3.14*radius*(degrees/360)
                print("Arclength", arclength)
                speed = arclength/totalTime # mm / s

                # Now that we have the speed, let's calculate the forward kinematics
                # First, we'll find the steering angle for the central wheel, using 
                # the radius we are given
                phi = math.atan(carLength_mm/radius) # in radians

                # Use steering angle to find rate of change of heading
                # Use equations of motion to figure out what the desired X,Y,Theta is
                thetaDot = (speed*math.tan(phi))/carLength_mm #radians per second
                
                # We will have totalTime/timeStep waypoints.
                # If we have 10 seconds, and a timestep of 1 second, we should 
                # have 10 waypoints.
                for i in range(int(totalTime/timeStep)):
                    x.append(previousX + speed*math.cos(theta[-1])*timeStep)
                    y.append(previousY + speed*math.sin(theta[-1])*timeStep)

                    # Theta is in degrees
                    theta.append(theta[-1] + thetaDot*timeStep)
                    
                    #Update where we are right now
                    previousX = x[-1]
                    previousY = y[-1]
                    time.append(time[-1] + timeStep)
        
        # print(time[-1])
        # plt.plot(x,y)
        # plt.show()
        # Populate trajectory message type
        self.Trajectory.X = x
        self.Trajectory.Y = y
        self.Trajectory.Theta = theta
        self.Trajectory.time = time

        self.TrajectoryMessagePublisher.publish(self.Trajectory)

    def forwardCallback(self):
        if self.Mode == "Tele-Op":
            print("Pressed Up")
            self.KeyboardMessage.publish(0)
    def backwardCallback(self):
        if self.Mode == "Tele-Op":
            print("Pressed Down")
            self.KeyboardMessage.publish(1)
    def leftCallback(self):
        if self.Mode == "Tele-Op":
            print("Pressed Left")
            self.KeyboardMessage.publish(2)

    def rightCallback(self):
        if self.Mode == "Tele-Op":
            print("Pressed Right")
            self.KeyboardMessage.publish(3)

    def brakeCallback(self):
        if self.Mode == "Tele-Op":
            print("Brake")
            self.KeyboardMessage.publish(4)
    def straightCallback(self):
        if self.Mode == "Tele-Op":
            print("Straight")
            self.KeyboardMessage.publish(5)


    def ChangeMode(self):
        self.Mode = self.ModeCombo.currentText()
        if(self.Mode == "Tele-Op"):
            self.RobotMode.publish(0)
            self.ForwardBTN.setEnabled(True)
            self.BackwardBTN.setEnabled(True)
            self.LeftBTN.setEnabled(True)
            self.RightBTN.setEnabled(True)
            self.BrakeBTN.setEnabled(True)
            self.StraightBTN.setEnabled(True)
            self.SendVandWBTN.setEnabled(False)
            self.AddArcBTN.setEnabled(False)
            self.AddLineBTN.setEnabled(False)
            self.SendSquareBTN.setEnabled(False)
            self.SendCustomTrajectoryBTN.setEnabled(False)
        elif(self.Mode == "Test Mode"):
            self.RobotMode.publish(1)
            self.ForwardBTN.setEnabled(False)
            self.BackwardBTN.setEnabled(False)
            self.LeftBTN.setEnabled(False)
            self.RightBTN.setEnabled(False)
            self.BrakeBTN.setEnabled(False)
            self.StraightBTN.setEnabled(False)
            self.SendVandWBTN.setEnabled(True)
            self.AddArcBTN.setEnabled(False)
            self.AddLineBTN.setEnabled(False)
            self.SendSquareBTN.setEnabled(False)
            self.SendCustomTrajectoryBTN.setEnabled(False)
        elif(self.Mode =="I/O Controller"):
            self.RobotMode.publish(2)
            self.ForwardBTN.setEnabled(False)
            self.BackwardBTN.setEnabled(False)
            self.LeftBTN.setEnabled(False)
            self.RightBTN.setEnabled(False)
            self.BrakeBTN.setEnabled(False)
            self.StraightBTN.setEnabled(False)
            self.SendVandWBTN.setEnabled(False)
            self.AddArcBTN.setEnabled(True)
            self.AddLineBTN.setEnabled(True)
            self.SendSquareBTN.setEnabled(True)
            self.SendCustomTrajectoryBTN.setEnabled(True)

    def SendWandVCallback(self):
        if self.Mode =="Test Mode":
            try:
                V = float(self.SetV.text())
            except:
                V = 0
                self.SetV.setText("0")
                print("Couldnt Parse V")
            try:
                W = float(self.SetW.text())

            except:
                W =0
                self.SetW.setText("0")
                print("Couldnt Parse W")
            vals = self.limiter(V, W)
            print(vals)
            self.SetV.setText(str(vals[0]))
            self.SetW.setText(str(vals[1]))
            if V==0 or W==0:
                print("Cant send no speed")
            else:
                sendVandW = rospy.ServiceProxy('VandW',VandWService)
                response = sendVandW(vals[0], vals[1])
                print(response)

    def limiter(self, V, W):
        # This takes in the commanded V and W from the controller, and limits the values to
        # values that the car can actually do
        # Use W to calculate psi, then limit psi
        # Use new psi to calculate W
        # Limit V to feasible value and send it
        carLength = 212
        carWidth  = 224
        limitedV = V
        velocityLimit  = 50 #2 inches persecond
        if(V > velocityLimit):
            limitedV = velocityLimit
        elif(V < -velocityLimit):
            limitedV = -velocityLimit
        psiTemp = math.atan(W*carLength/V)
        if(psiTemp > math.radians(35)):
            psiTemp = math.radians(35)
        if(psiTemp < math.radians(-35)):
            psiTemp = math.radians(-35)
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