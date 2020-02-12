from ADC import *
from Motor import *
from Encoder import *
from enum import Enum
#TODO: Have an enumeration for car pins,
# and then initialize a motor (with an encoder),
# and a ADC in the constructor
class carParams(Enum):
    driveMotorPWMPin = 5
    driveMotorDirPin = 6
    steeringMotorPWMPin = 13
    steeringMotorDirPin = 26

class Car(): 
    def __init__(self, address = None):
        self.adc = ADC()
        self.steeringMotor = Motor(carParams.steeringMotorDirPin.value, carParams.steeringMotorPWMPin.value)
        #put an encoder on this one
        self.drivingMotor = Motor(carParams.driveMotorDirPin.value, carParams.driveMotorPWMPin.value, encoderA=21, encoderB=20)
        pass

    def driveForward(self, distance):
	    pass

    def voltageToAngle(self, voltage):
        #takes voltage value and turns it into an angle
        #returns the angle
        print((45/1.65 * voltage) - 45)
        return (45/1.65 * voltage) - 45

    def turnRightRelative(self, degrees):
        print("Turning Right")
        self.steeringMotor.setDirection(1)
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        newAngle = currentAngle
        while(newAngle > (currentAngle - degrees)):
            self.steeringMotor.turn(-90)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        self.steeringMotor.turn(0)
        print("Turned Right")

    def turnLeftRelative(self, degrees):
        print("Turning Left")
        self.steeringMotor.setDirection(0)
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        newAngle = currentAngle
        while (newAngle < (degrees + currentAngle)):
            self.steeringMotor.turn(90)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        self.steeringMotor.turn(0)
        print("Turned Left")

    def turnAbsolute(self, degree):
        currentAngle =  self.voltageToAngle(self.adc.AnalogRead(0).voltage)

        #Angle is to the right
        if degree < currentAngle:
            self.turnRightRelative(currentAngle - degree)

        #angle is to the left
        elif degree > currentAngle:
            self.turnLeftRelative(degree - currentAngle)

        print("Went to angle:", self.voltageToAngle(self.adc.AnalogRead(0).voltage))

c = Car()
#while(1):
    #print(c.voltageToAngle(c.adc.AnalogRead(0).voltage))
# c.turnRightRelative(10)
# time.sleep(1.5)
c.turnLeftRelative(10)