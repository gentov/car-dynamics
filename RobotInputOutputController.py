#!/usr/bin/env python3
import numpy as np
import math
import rospy
import time as timer
from carpackage.srv import VandWService

from carpackage.msg import TrajectoryMSG

class InputOutputController:
    def __init__(self):
        # Data structure to hold trajectory information
        # This list contains smaller lists [x,y,theta, time]
        # It is ordered so element 1 is the first 'waypoint' on the trajectory

        # We have a dictionary of times and desired poses
        # runController will take in a time, ceiling it, and then use that time to access the desired pose
        self.trajectoryWaypoints = {}

        # Initial Conditions (X = 0, Y = 0, Theta = 0)
        self.startingPose = [0, 0, 0]

        # Current Position
        self.currentPose = [0,0,0]

        # Point we are tracking (length of car) (don't remember)
        self.b = 212

        #keeps track of elapsed time
        self.time = timer.time()
        self.StartTime = timer.time()

        # The two gains
        self.K1 = 2
        self.K2 = 2

        # The control inputs
        self.V = None
        self.W = None

        self.sentTrajectory = rospy.Subscriber('/Trajectory', TrajectoryMSG, self.setTrajectoryAndRun, queue_size=1)
        self.VandWService = rospy.wait_for_service("VandW")
        print("Started")
    def setTrajectoryAndRun(self, data):
        self.trajectoryWaypoints = {}
        # This is the callback from the subscriber
        # It populates the trajectoryWaypoints data
        print("Recieved Trajectory")
        # Grab the number of waypoints from the trajectory message (all X,Y, Theta, time arrays should be same length)
        numberOfWaypoints = len(data.X)
        for i in range(numberOfWaypoints):
            # This essentially gives us access to desired X, Desired Y, and desired Theta at time t
            time = data.time[i]
            pose = [data.X[i], data.Y[i], data.Theta[i]]
            self.trajectoryWaypoints[time] = pose
        self.time = timer.time()
        self.LastTime = timer.time()
        self.StartTime = timer.time()
        print(self.trajectoryWaypoints)
        self.runController()



    def runController(self):
        #run the error dynamics controller

        # while(1)
        # Take the current X,Y,theta, and calculate desired V and W. Send to robot, this V and W. Request actual V and W
        # Use actual V and W to update current Pose. Use current Pose to calculate new desired V and W
        self.currentPose[0] = self.startingPose[0]
        self.currentPose[1] = self.startingPose[1]
        self.currentPose[2] = self.startingPose[2]


        while(1):

            #round the class attribute time to feed it to the trajectoryWaypoints Data structure
            time = round(self.LastTime - self.StartTime, 1)
            print("time" + str(time))
            #print("time "+str((self.LastTime - self.StartTime)))
            x = self.currentPose[0]
            y = self.currentPose[1]
            theta = self.currentPose[2]
            try:
                desiredPose = self.trajectoryWaypoints[time]

            except:
                print("Time doesnt exist, End of trajectory")
                sendVandW = rospy.ServiceProxy('VandW', VandWService)
                response = sendVandW(0, 0) # stops robot
                ActualV = response.Vactual
                ActualW = round(response.Wactual, 2)  # The controller updates too slowly to adjust for tiny errors in W
                dt = timer.time() - self.LastTime
                print("Desired State")
                print(str(self.V) + " mm/s " + str(self.W) + " rad/s")
                print("Robot State")
                print(str(ActualV) + " mm/s " + str(ActualW) + " rad/s")
                # Update the currentPose
                self.currentPose[2] = theta + ActualW * dt
                self.currentPose[0] = x + ActualV * math.cos(self.currentPose[2]) * dt
                self.currentPose[1] = y + ActualV * math.sin(self.currentPose[2]) * dt

                self.LastTime = timer.time()
                print("Current Pose")
                print(self.currentPose)
                print("Desired Pose")
                print(desiredPose)
                break


            # Desired X, Y, Theta
            desiredX = desiredPose[0]
            desiredY = desiredPose[1]
            desiredTheta = desiredPose[2]

            # Finding control inputs
            y1 = x + self.b*math.cos(theta)
            y2 = y + self.b*math.sin(theta)

            y1Des = desiredX + self.b*math.cos(desiredTheta)
            y2Des = desiredY + self.b*math.sin(desiredTheta)

            u1 = self.K1*(y1Des - y1)
            u2 = self.K2*(y2Des - y2)

            # Vector to hold U1 and U2
            U = np.array([[u1],[u2]])

            T = np.array([[math.cos(theta), math.sin(theta)],
                          [-math.sin(theta)/self.b, math.cos(theta)/self.b]])

            controlInputs = np.matmul(T, U)

            # Desired V an W
            V = controlInputs[0]
            W = controlInputs[1]
            #print("Before Limit")
            #print(V)
            #print(W)
            vals = self.limiter(V, W)
            #print("After Limit")
            #print(vals)

            self.V = vals[0]
            self.W = vals[1]
            # Send V and W to the robot
            #responseTimeStart =timer.time()
            sendVandW = rospy.ServiceProxy('VandW',VandWService)

            response = sendVandW(self.V, self.W)
            #responseTimeEnd = timer.time()
            #print("Time to get response: " + str(responseTimeEnd - responseTimeStart))
            ActualV = response.Vactual
            ActualW = round(response.Wactual, 2)#The controller updates too slowly to adjust for tiny errors in W
            dt = timer.time()-self.LastTime
            self.LastTime = timer.time()
            print("Desired State")
            print(str(self.V) + " mm/s " + str(self.W) + " rad/s")
            print("Robot State")
            print(str(ActualV)+" mm/s "+ str(ActualW)+" rad/s")
            #Update the currentPose
            self.currentPose[2] = theta + ActualW * dt
            self.currentPose[0] = x + ActualV*math.cos( self.currentPose[2])*dt
            self.currentPose[1] = y + ActualV*math.sin( self.currentPose[2])*dt


            print("Current Pose")
            print(self.currentPose)
            print("Desired Pose")
            print(desiredPose)


    def limiter(self, V, W):
        # This takes in the commanded V and W from the controller, and limits the values to
        # values that the car can actually do
        # Use W to calculate psi, then limit psi
        # Use new psi to calculate W
        # Limit V to feasible value and send it
        if V == 0:
            limitedV = 0#if no speed, straighten out wheels
            limitedW = 0
        else:
            carLength = 212
            carWidth  = 224
            limitedV = V
            velocityLimit  = 100
            if(V > velocityLimit):
                limitedV = velocityLimit
            elif(V < -velocityLimit):
                limitedV = -velocityLimit
            psiTemp = math.atan(W*carLength/limitedV)
           # print("Psi Temp")
           # print(math.degrees(psiTemp))

            if(psiTemp > math.radians(25)):
                psiTemp = math.radians(25)
             #   print("Too Large")
            if(psiTemp < math.radians(-25)):

                psiTemp = math.radians(-25)
            #    print("Too Small")
            limitedW = float(math.tan(psiTemp)/carLength*limitedV)

        return(limitedV, limitedW)



if __name__ == '__main__':
    rospy.init_node('InputOutputController')
    rospy.sleep(.5)
    controller = InputOutputController()
    while not rospy.is_shutdown():
        pass