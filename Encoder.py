import RPi.GPIO as GPIO
import time

class Encoder():
    def __init__(self, ChA, ChB, TicksPerRevolution):
        GPIO.setmode(GPIO.BOARD)
        self.ChA = ChA
        self.ChB = ChB
        GPIO.setup(ChA, GPIO.IN)
        GPIO.setup(ChB, GPIO.IN)
        self.dir = 0
        GPIO.add_event_detect(self.ChA, GPIO.RISING, callback=self.TickCountA)
        GPIO.add_event_detect(self.ChB, GPIO.RISING, callback=self.TickCountB)
        self.TicksToAngleRadians = 6.283/float(TicksPerRevolution)
        self.ticks = 0
        self.Angle = 0
        self.previousTime = time.time()
        self.velocity = 0.0
        self.lastTickA = False
        self.lastTickB = False

    def TickCountA(self, data):
        currentTime = time.time()
        if (self.lastTickA == True):
            self.ticks += 0
        elif(GPIO.input(self.ChB)):
            self.dir = 1
            self.ticks -= 1
        else:
            self.dir = 0
            self.ticks += 1

        if(self.dir==0):
            self.velocity = self.TicksToAngleRadians/(currentTime-self.previousTime)
        else:
            self.velocity = -self.TicksToAngleRadians / (currentTime - self.previousTime)
        self.Angle = float(self.ticks)*self.TicksToAngleRadians
        self.previousTime = currentTime
        self.lastTickA = True
        self.lastTickB = False

    def TickCountB(self, data):

        if(self.lastTickB == True):
            self.ticks += 0
        elif(GPIO.input(self.ChA)):
            self.dir = 0
            self.ticks += 1
        else:
            self.dir = 1
            self.ticks -= 1


        self.Angle = float(self.ticks)*self.TicksToAngleRadians
        self.lastTickA = False
        self.lastTickB = True
