from ADC import *
from Motor import *
from Encoder import *
#TODO: Have an enumeration for car pins,
# and then initialize a motor (with an encoder),
# and a ADC in the constructor
class Car(): 
    def __init__(self, address = None):
        self.adc = ADC()
        self.steeringMotor = Motor()
        #put an encoder on this one
        self.drivingMotor = Motor()
        pass

    def driveForward(self, distance):
	    pass

    def voltageToAngle(self, voltage):
        #takes ADC value and turns it into an angle
        # returns the angle
        pass

    def turnRightRelative(self, degrees):
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).value)
        newAngle = currentAngle
        while(abs(newAngle - currentAngle) < degrees):
            self.steeringMotor.turn(10)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).value)

    def turnLeftRelative(self, degrees):
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).value)
        newAngle = currentAngle
        while (abs(newAngle - currentAngle) < degrees):
            self.steeringMotor.turn(-10)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).value)

    def turnAbsolute(self, degree):
        currentAngle =  self.voltageToAngle(self.adc.AnalogRead(0).value)

        #Angle is to the right
        if degree < currentAngle:
            self.turnRightRelative(currentAngle - degree)

        #angle is to the left
        elif degree > currentAngle:
            self.turnLeftRelative(degree - currentAngle)

        print("Went to angle:", self.voltageToAngle(self.adc.AnalogRead(0).value))
