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
        self.ticks = 0
        self.lastTickA = False
        self.lastTickB = False

    def TickCountA(self, data):
        #if (self.lastTickA == True):
        #    self.dir = ~self.dir
        if(self.dir):
            self.ticks -= 1
        else:
            self.ticks += 1
        #self.lastTickA = True
        #self.lastTickB = False

    def TickCountB(self, data):
        if(self.lastTickB == True):
            self.dir = ~self.dir
        if(self.dir):
            self.ticks += 1
        else:
            self.ticks -= 1
        self.lastTickA = False
        self.lastTickB = True
