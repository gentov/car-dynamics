from ADC import *
from Motor import *
from Encoder import *
from enum import Enum
import time
#TODO: Have an enumeration for car pins,
# and then initialize a motor (with an encoder),
# and a ADC in the constructor
class carParams(Enum):
    driveMotorPWMPin = 16
    driveMotorDirPin = 12
    steeringMotorPWMPin = 26
    steeringMotorDirPin = 13

class Car(): 
    def __init__(self, address = None):
        self.adc = ADC()
        self.steeringMotor = Motor(carParams.steeringMotorDirPin.value, carParams.steeringMotorPWMPin.value)
        #put an encoder on this one
        self.drivingMotor = Motor(carParams.driveMotorDirPin.value, carParams.driveMotorPWMPin.value, encoderA=23, encoderB=24, encoderTicksPerRevolution=100)
        self.maxSpeed = 100 # mm/s (needs updating)
        self.DesiredSteeringAngle = 0
        self.AngleTolerance = 0.05
        self.SteeringThread = threading.Thread(target=self.turnToDesiredAngle)
        
        self.steeringSpeed = 20
        self.CalcVelocityThread = threading.Thread(target = self.getLinearVelocity)
       
        self.steeringAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        self.carLength = .212
        self.carWidth = .224
        self.LastEncoderTicks = 0
        self.TimeOfLastCheck = 0
        self.TimeOfNextCheck = 0
        self.LinearVelocity = 0
        self.VelCheckFrequency = 10 #10 times per second
        self.TicksToMM = 100 #needs updating
        self.SteeringThread.start()
        self.CalcVelocityThread.start()
        
    def getLinearVelocity():
        while(True):
            if self.TimeOfNextCheck<time.time():
                currentTicks = self.encoder.ticks
                changeInTicks = currentTicks-self.LastEncoderTicks
                currentTime = time.time()
                changeInTime = currentTime-self.TimeOfLastCheck
                self.LinearVelocity = changeInTicks/changeInTime*self.TicksToMM
                self.LastEncoderTicks = currentTicks
                self.TimeOfLastCheck = currentTime
                self.TimeOfNextCheck = currentTime+1/self.VelCheckFrequency    

    def voltageToAngle(self, voltage):
        #takes voltage value and turns it into an angle
        #returns the angle
        print((45/1.65 * voltage) - 45)
        return (45/1.65 * voltage) - 45

    def turnRightRelative(self, degrees):
        print("Turning Right")
        self.steeringMotor.setDirection(0)
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        newAngle = currentAngle
        while(newAngle > (currentAngle - degrees)):
            self.steeringMotor.turn(self.steeringSpeed)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        self.steeringMotor.turn(0)
        print("Turned Right")

    def turnLeftRelative(self, degrees):
        print("Turning Left")
        self.steeringMotor.setDirection(1)
        currentAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        newAngle = currentAngle
        while (newAngle < (degrees + currentAngle)):
            self.steeringMotor.turn(self.steeringSpeed)
            newAngle = self.voltageToAngle(self.adc.AnalogRead(0).voltage)
        self.steeringMotor.turn(0)
        print("Turned Left")

    def turnAbsolute(self, degree):
        currentAngle =  self.voltageToAngle(self.adc.AnalogRead(0).voltage)

        #Angle is to the right
        if degree < currentAngle:
            self.turnRightRelative(abs(currentAngle - degree))

        #angle is to the left
        elif degree > currentAngle:
            self.turnLeftRelative(abs(degree - currentAngle))

        print("Went to angle:", self.voltageToAngle(self.adc.AnalogRead(0).voltage))

    def turnToDesiredAngle():
        while(True):
            self.steeringAngle =  self.voltageToAngle(self.adc.AnalogRead(0).voltage)
            if self.DesiredSteeringAngle < (self.steeringAngle - self.AngleTolerance):
                self.steeringMotor.setDirection(0)
                self.steeringMotor.turn(self.steeringSpeed)
            elif self.DesiredSteeringAngle > (self.steeringAngle + self.AngleTolerance):
                self.steeringMotor.setDirection(1)
                self.steeringMotor.turn(self.steeringSpeed)
            else:
                self.steeringMotor.turn(0)

    def getAngularVelocity():
        self.AngularVelocity = (self.getLinearVelocity*math.tan(self.steeringAngle))/self.carLength; #ThetaDot in radians
        pass



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