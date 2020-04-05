from ADC import *
from Motor import *
from Encoder import *
from enum import Enum
import time
import threading
import math
import smbus
#TODO: Have an enumeration for car pins,
# and then initialize a motor (with an encoder),
# and a ADC in the constructor
class carParams(Enum):
    driveMotorPWMPin = 16
    driveMotorDirPin = 12
    steeringMotorPWMPin = 26
    steeringMotorDirPin = 13

class Car(): 
    def __init__(self):

        self.bus = smbus.SMBus(1)
        self.addressUno = 0x08
        self.adc = ADC()
        self.steeringMotor = Motor(carParams.steeringMotorDirPin.value, carParams.steeringMotorPWMPin.value)
        #put an encoder on this one
        self.drivingMotor = Motor(carParams.driveMotorDirPin.value, carParams.driveMotorPWMPin.value, encoderA=23, encoderB=24, encoderTicksPerRevolution=100)
        self.maxSpeed = 100 # mm/s (needs updating)
        self.DesiredSteeringAngle = 0
        self.AngleTolerance = 0.1
        
        self.steeringSpeed = 12
        self.CalcVelocityThread = threading.Thread(target = self.updateStates)
        self.steeringDirection = 0
        self.steeringMoving = False
       
        self.steeringAngle = 0
        self.carLength = .212
        self.carWidth = .224
        self.ticks = 0
        self.LastEncoderTicks = 0
        self.TimeOfLastCheck = 0
        self.TimeOfNextCheck = 0
        self.LinearVelocity = 0
        self.AngularVelocity = 0
        self.VelCheckFrequency = 10 #10 times per second
        self.TicksToMM = 1.180 #needs updating
        self.CalcVelocityThread.start()
        
    def updateStates(self):
        while(True):
            if self.TimeOfNextCheck<time.time():
                self.retrieveMessage()
                changeInTicks = self.ticks-self.LastEncoderTicks
                currentTime = time.time()
                changeInTime = currentTime-self.TimeOfLastCheck
                self.LinearVelocity = changeInTicks/changeInTime/self.TicksToMM
                self.LastEncoderTicks = self.ticks
                self.TimeOfLastCheck = currentTime
                self.TimeOfNextCheck = currentTime+1/self.VelCheckFrequency
                self.AngularVelocity = self.getAngularVelocity()

    def adcToAngle(self, ADC):
        #takes the adc value and converts it into an angle value
        angle = (ADC/11.38) - 45
        print(angle)
        return angle
    
    def angleToADC(self, angle):
        #takes and angle value and converts it into an ADC
        ADC = (11.38 * (angle + 45))
        print(ADC)
        return ADC

    def turnToDesiredAngle(self, angle):
        msg = [1, self.angleToADC(self.steeringAngle)]
        self.sendMessage(msg)
        # while(True):
        #     self.steeringAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        #     if self.DesiredSteeringAngle < (self.steeringAngle - self.AngleTolerance):
        #         if self.steeringDirection!=0:
        #             self.steeringMotor.setDirection(0)
        #             self.steeringDirection = 0

        #         if self.steeringMoving == False:
        #             self.steeringMotor.turn(self.steeringSpeed)
        #             self.steeringMoving = True
        #     elif self.DesiredSteeringAngle > (self.steeringAngle + self.AngleTolerance):
        #         if self.steeringDirection!=1:
        #             self.steeringMotor.setDirection(1)
        #             self.steeringDirection = 1
        #         if self.steeringMoving == False:
        #             self.steeringMotor.turn(self.steeringSpeed)
        #             self.steeringMoving = True
        #     else:
        #         self.steeringMotor.turn(0)
        #         self.steeringMoving = False
    
    def setMotor(self, speed, direction):
        msg = [2, speed, direction]
        self.sendMessage(msg)

    def getAngularVelocity(self):
        self.AngularVelocity = (self.LinearVelocity*math.tan(self.steeringAngle))/self.carLength 
        return self.AngularVelocity

    def sendMessage(self, msg):
        self.bus.write_i2c_block_data(self.addressUno, 3, msg)

    def retrieveMessage(self):
        state = self.bus.read_i2c_block_data(self.addressUno, 8)
        ## Convert the state to a V and an W based on ticks and steering angle
        ADCValue = state[0]
        self.ticks = state[1]
        self.steeringAngle = self.adcToAngle(ADCValue)
        
        

if __name__ == '__main__':
    c = Car()
    c.drivingMotor.turn(100)
    #while(1):
        #print(c.voltageToAngle(c.adc.AnalogRead(0).voltage))
    # c.turnRightRelative(10)
    # time.sleep(1.5)
    # c.turnLeftRelative(45)
    # c.turnRightRelative(45)
    #c.turnAbsolute(0)