#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from Robot import *
from carpackage.srv import VandWService
import math

class CarController:
    def __init__(self):
        self.KeyboardMessage = rospy.Subscriber('/CarKeyboard', Float64, self.keyboardCallback, queue_size=1)
        self.RobotMode = rospy.Subscriber('/CarMode', Float64, self.ChangeMode, queue_size=1)
        self.car = Car()
        self.car.turnAbsolute(0)
        self.carmode = 0 #0 is for teleop 1 is for autonomous
        serviceVandW = rospy.Service("VandW", VandWService, self.updateVandW)
        
    def ChangeMode(self, data):
        self.carmode = data.data

    def updateVandW(self, data):
        if self.carmode == 1:
            pass
        # Calculate steering angle from W
        self.car.DesiredSteeringAngle = math.atan(W*carLength/V)
       
        if(V < 0):
            self.car.drivingMotor.setDirection(1)
        elif(V > 0):
            self.car.drivingMotor.setDirection(0)
        
        self.car.drivingMotor.turn(self.velocityToPWM(V))
        
        # Set drive motor to V
        #waits some amount of time
        #returns measured V and W
        measuredV = self.car.getLinearVelocity()
        measuredW = self.car.getAngularVelocity()
        timeElapsed = 0 #add function
        return VandWServiceResponse(measuredV, measuredW, timeElapsed)

    def velocityToPWM(self, velocity)
        maxPWM = 100
        velocityToPWMRatio = maxPWM/self.car.maxSpeed
        return velocity * velocityToPWMRatio
    
    def keyboardCallback(self, data):
        if self.carmode == 0:
            if data.data == 0:
                print("Pressed Up")
                self.car.drivingMotor.setDirection(0)
                self.car.drivingMotor.turn(100)
            elif data.data == 1:
                print("Pressed Down")
                self.car.drivingMotor.setDirection(1)
                self.car.drivingMotor.turn(100)
            elif data.data == 2:
                print("Pressed Left")
                self.car.DesiredSteeringAngle = 44

            elif data.data == 3:
                print("Pressed Right")
                self.car.DesiredSteeringAngle = -44

            elif data.data == 4:
                print("Stop")
                self.car.drivingMotor.turn(0)
            elif data.data == 5:
                self.car.DesiredSteeringAngle = 0
            elif data.data == 6:
                print("Velocity: ", self.car.drivingMotor.encoder.velocity)
                print("Ticks: ", self.car.drivingMotor.encoder.ticks)
            else:
                print(data)


if __name__ == '__main__':
    rospy.init_node('Car_Controller')
    rospy.sleep(.5)
    car = CarController()
    while not rospy.is_shutdown():
        pass