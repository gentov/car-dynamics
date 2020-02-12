import RPi.GPIO as GPIO
import time
from Encoder import *
class Motor():
    Frequency = 250
    StopPWM = 0

    def __init__(self, dirPin, pwmPin,encoderA = None, encoderB =  None, encoderTicksPerRevolution = None):
        GPIO.setmode(GPIO.BCM)
        if(encoderA is not None and encoderB is not None and encoderTicksPerRevolution is not None):
            self.encoder = Encoder(encoderA, encoderB, encoderTicksPerRevolution)
        self.dirPin = dirPin
        self.PWMPin = pwmPin
        GPIO.setup(self.dirPin, GPIO.OUT)
        GPIO.setup(self.PWMPin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.PWMPin, self.Frequency)
        self.speed = self.StopPWM
        self.verbose = False
        self.counterclockwise = 1
        self.clockwise = 0

    #As a percentage 0 - 100
    def turn(self, speed):
        if(speed == 0):
            #GPIO.output(self.dirPin, self.counterclockwise)
            time.sleep(.002)
            self.pwm.start(self.StopPWM)
            time.sleep(.002)

        elif speed>0.0:
            time.sleep(.002)
            #GPIO.output(self.dirPin, self.counterclockwise)
            if speed>100:
                speed = 100
            self.pwm.start(speed)
            if(self.verbose == True):
                print("Speed:", speed)
            time.sleep(.002)

        elif speed<0.0:
            time.sleep(.002)
            #GPIO.output(self.dirPin, self.clockwise)
            if(abs(speed) > 100):
                speed = 100
            self.pwm.start(abs(speed))
            if(self.verbose == True):
                print("Speed: ",speed)
            time.sleep(.002)
