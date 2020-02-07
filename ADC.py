import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class ADC():
    pins = {
	0: "ADS.P0",
	1: "ADS.P1",
	2: "ADS.P2",
	3: "ADS.P3"
	} 
    def __init__(self, address = None):
        if address is not None:
            self.address = address
            i2c = ads = ADS.ADS1115(i2c, address=0x4a)
        else:
            i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(i2c)

    def AnalogRead(self, channel):
	#Returns a channel object
	#Attributes: Voltage (voltage), and ADC Value (value)
        return AnalogIn(self.ads, eval(self.pins[channel]))

    def setGain(self, gain):
        self.ads.gain = gain

    def getGain(self):
        return self.ads.gain

    def printAnalogReadings(self):
        for i in range(len(self.pins)):
            print("Pin:", i, ", Value:", self.AnalogRead(i).value, ", Voltage:", self.AnalogRead(i).voltage) 

a = ADC()
a.printAnalogReadings()
