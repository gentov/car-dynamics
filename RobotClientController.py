#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from Robot import *
from carpackage.srv import VandWService, VandWServiceResponse
import math

class CarController:
    def __init__(self):
        self.KeyboardMessage = rospy.Subscriber('/CarKeyboard', Float64, self.keyboardCallback, queue_size=1)
        self.RobotMode = rospy.Subscriber('/CarMode', Float64, self.ChangeMode, queue_size=1)
        self.car = Car()
        self.carmode = 0 #0 is for teleop 2 is for autonomous, 1 is for test
        self.carLength = .212
        self.carWidth = .224
        serviceVandW = rospy.Service("VandW", VandWService, self.updateVandW)
        
    def ChangeMode(self, data):
        self.carmode = data.data

    def updateVandW(self, data):
        measuredV = 0
        measuredW = 0
        timeElapsed = 0
        if self.carmode == 1 or self.carmode == 2:
            # Calculate steering angle from W, thread actually moves it
            self.car.DesiredSteeringAngle = math.atan(data.W*self.carLength/data.V)
        
            if(data.V < 0):
                self.car.setMotor(self.velocityToPWM(data.V), 1)
            elif(data.V > 0):
                self.car.setMotor(self.velocityToPWM(data.V), 0)

            # Set drive motor to V
            #waits some amount of time
            #returns measured V and W
            measuredV = self.car.LinearVelocity
            measuredW = self.car.AngularVelocity
            timeElapsed = 0 #add function
        return VandWServiceResponse(measuredV, measuredW, timeElapsed)

    def velocityToPWM(self, velocity):
        maxPWM = 255
        velocityToPWMRatio = maxPWM/self.car.maxSpeed
        return velocity * velocityToPWMRatio
    
    def keyboardCallback(self, data):
        if self.carmode == 0:
            if data.data == 0:
                print("Pressed Up")
                self.car.setMotor(255, 0)

            elif data.data == 1:
                print("Pressed Down")
                self.car.setMotor(255, 1)

            elif data.data == 2:
                print("Pressed Left")
                self.car.turnToDesiredAngle(-35)


            elif data.data == 3:
                print("Pressed Right")
                self.car.turnToDesiredAngle(35)

            elif data.data == 4:
                print("Stop")
                self.car.setMotor(0, 1)
            elif data.data == 5:
                self.car.turnToDesiredAngle(0)
            elif data.data == 6:
                print("Velocity: ", self.car.LinearVelocity)
                print("Ticks: ", self.car.ticks)
            else:
                print(data)


if __name__ == '__main__':
    rospy.init_node('Car_Controller')
    rospy.sleep(.5)
    car = CarController()
    while not rospy.is_shutdown():
        pass