
import numpy as np
import math
import rospy
from carpackage.srv import VandWService
from std_msgs.msg import Float64
class InputOutputController():
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
        self.time = 0

        # The two gains
        self.K1 = 8
        self.K2 = 8

        # The control inputs
        self.V = None
        self.W = None

        self.sentTrajectory = rospy.Subscriber('/TrajectoryMSG', Float64, self.setTrajectoryAndRun, queue_size=1)
        self.VandWService = rospy.wait_for_service("VandW")

    def setTrajectoryAndRun(self, data):
        # This is the callback from the subscriber
        # It populates the trajectoryWaypoints data

        # Grab the number of waypoints from the trajectory message (all X,Y, Theta, time arrays should be same length)
        numberOfWaypoints = len(data.X)
        for i in range(numberOfWaypoints):
            # This essentially gives us access to desired X, Desired Y, and desired Theta at time t
            time = data.time[i]
            pose = [data.X[i], data.Y[i], data.Theta[i]]
            self.trajectoryWaypoints[time] = pose

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
            time = round(self.time)

            x = self.currentPose[0]
            y = self.currentPose[1]
            theta = self.currentPose[2]

            desiredPose = self.trajectoryWaypoints[time]
            T = np.array([[math.cos(theta), -self.b*math.sin(theta)],
                        [math.sin(theta), self.b*math.cos(theta)]])

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


            invT = np.linalg.inv(T)

            controlInputs = invT*U

            # Desired V an W
            V = controlInputs[0]
            W = controlInputs[1]

            self.V, self.W = self.limiter(V,W)

            # Send V and W to the robot
            sendVandW = rospy.ServiceProxy('VandW',VandWService)
            response = sendVandW(self.V, self.W)

            ActualV = response.Vactual
            ActualW = response.Wactual
            dt = response.timeStep

            #Update the currentPose
            self.currentPose[0] = x + ActualV*math.cos(theta)*dt
            self.currentPose[1] = y + ActualV*math.sin(theta)*dt
            self.currentPose[2] = theta + ActualW*dt
            self.time = time + dt


    def limiter(self, V, W):
        # This takes in the commanded V and W from the controller, and limits the values to
        # values that the car can actually do
        # Use W to calculate psi, then limit psi
        # Use new psi to calculate W
        # Limit V to feasible value and send it
        carLength = 212
        carWidth  = 224
        limitedV = V
        velocityLimit  = 50
        if(V > velocityLimit):
            limitedV = velocityLimit
        elif(V < -velocityLimit):
            limitedV = -velocityLimit
        psiTemp = math.atan(W*carLength/V)
        if(psiTemp > math.radians(45)):
            psiTemp = math.radians(45)
        if(psiTemp < math.radians(-45)):
            psiTemp = math.radians(-45)
        limitedW = math.tan(psiTemp)/carLength*V

        return(limitedV,limitedW)



