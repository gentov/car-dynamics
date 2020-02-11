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
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        newAngle = currentAngle
        while(abs(newAngle - currentAngle) < degrees):
            self.steeringMotor.turn(1)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)

    def turnLeftRelative(self, degrees):
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        newAngle = currentAngle
        while (abs(newAngle - currentAngle) < degrees):
            self.steeringMotor.turn(-1)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)

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
c.turnRightRelative(10)
time.sleep(1.5)
c.turnLeftRelative(10)
time.sleep(1.5)