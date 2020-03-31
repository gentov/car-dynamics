import RPi.GPIO as GPIO
import time

class Encoder():
    def __init__(self, ChA, ChB, TicksPerRevolution):
        GPIO.setmode(GPIO.BCM)
        self.ChA = ChA
        self.ChB = ChB
        GPIO.setup(ChA, GPIO.IN)
        GPIO.setup(ChB, GPIO.IN)
        self.dir = 0
        GPIO.add_event_detect(self.ChA, GPIO.RISING, callback=self.TickCountA)
        #GPIO.add_event_detect(self.ChB, GPIO.RISING, callback=self.TickCountB)
        self.TicksToMeters = float(.239)/float(TicksPerRevolution) #3" diameter wheels
        self.ticks = 0
        self.meters = 0
        self.previousTime = time.time()
        self.velocity = 0.0
        self.lastTickA = False
        self.lastTickB = False

    def TickCountA(self, data):
        #currentTime = time.time()
        #if (self.lastTickA == True):
        #    pass
        if(GPIO.input(self.ChB)):
            self.dir = 1
            self.ticks -= 1
        else:
            self.dir = 0
            self.ticks += 1

        #if(self.dir==0):
        #    self.velocity = self.TicksToMeters/(currentTime-self.previousTime)
        #else:
        #   self.velocity = -self.TicksToMeters / (currentTime - self.previousTime)
        self.meters = float(self.ticks)*self.TicksToMeters
        #self.previousTime = currentTime
        self.lastTickA = True
        self.lastTickB = False
        #print(self.velocity)

    def TickCountB(self, data):
        if(self.lastTickB == True):
            pass
        elif(GPIO.input(self.ChA)):
            self.dir = 0
            self.ticks += 1
        else:
            self.dir = 1
            self.ticks -= 1

        self.meters = float(self.ticks)*self.TicksToMeters
        self.lastTickA = False
        self.lastTickB = True
