import time
import threading
import math
import smbus
import serial

class Car(): 
    def __init__(self):

        self.bus = smbus.SMBus(1)
        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=3)
        self.addressUno = 0x04
        self.maxSpeed = 100 # mm/s (needs updating)
        self.DesiredSteeringAngle = 0
        self.AngleTolerance = 0.1
        self.steeringSpeed = 12
        #self.CalcVelocityThread = threading.Thread(target = self.updateStates)
        self.steeringDirection = 0
        self.steeringMoving = False
       
        self.steeringAngle = 0
        self.carLength = 212
        self.carWidth = 224
        self.ticks = 0
        self.LastEncoderTicks = 0
        self.TimeOfLastCheck = time.time()
        self.TimeOfNextCheck = 0
        self.LinearVelocity = 0
        self.AngularVelocity = 0
        self.VelCheckFrequency = 3 #10 times per second
        self.TicksToMM = 2.56 #needs updating
        self.busBusy = False
        self.updateStates()
        
    def updateStates(self):
        #while(self.runCalcThread):
            #if self.TimeOfNextCheck<time.time():

        self.retrieveMessageSerial()
        changeInTicks = self.ticks-self.LastEncoderTicks
        currentTime = time.time()
        changeInTime = currentTime-self.TimeOfLastCheck
        self.LinearVelocity = changeInTicks/changeInTime/self.TicksToMM
        self.LastEncoderTicks = self.ticks
        self.TimeOfLastCheck = currentTime
        self.TimeOfNextCheck = currentTime+1/self.VelCheckFrequency
        self.AngularVelocity = self.getAngularVelocity()
        print(self.ticks)
        #self.CalcVelocityThread = threading.Thread(target=self.updateStates)


    def adcToAngle(self, ADC):
        #takes the adc value and converts it into an angle value
        angle = (ADC/21.557) - 23.7
       # print(angle)
        return angle
    
    def angleToADC(self, angle):
        #takes and angle value and converts it into an ADC
        ADC = (21.557 * (angle + 23.7))
        #print(ADC)
        return ADC


    def turnToDesiredAngle(self, angle):
        ADC = str(int(self.angleToADC(math.degrees(angle))))
        print("ADC Value "+str(ADC))
        #temp = ADC.to_bytes(2, "big", signed=False)
        #msg = [int(temp[0]), int(temp[1])]
        #self.sendMessage(1, msg)
        msg = "1"
        for i in range(0,4-len(ADC)):
            msg = msg +"0"
        msg = msg + ADC
        self.sendMessageSerial(bytes(msg, "ascii"))
    
    def setMotor(self, speed, direction):
        #msg = [int(speed), direction]
        #self.sendMessage(2, msg)
        print("Motor Speed" +str(speed))
        msg = "2"
        msg = msg + str(direction)
        for i in range(0,3-len(str(speed))):
            msg = msg +"0"
        msg = msg + str(speed)
        self.sendMessageSerial(bytes(msg, "ascii"))

    def getAngularVelocity(self):
        self.AngularVelocity = (self.LinearVelocity*math.tan(self.steeringAngle))/self.carLength 
        return self.AngularVelocity

    def sendMessage(self, cmd, msg):
        self.bus.write_i2c_block_data(self.addressUno, cmd, msg)

    def sendMessageSerial(self, msg):
        self.ser.write(bytes(msg, 'ascii'))


    def retrieveMessage(self):
        if(self.busBusy==False):
            self.busBusy = True
            bytes = []
            data1 = self.bus.read_byte(self.addressUno)
            bytes.append(data1)
            data2 = self.bus.read_byte(self.addressUno)
            bytes.append(data2)
            ADCValue = int.from_bytes(bytes, "big")
            time.sleep(0.02)
            ## Convert the state to a V and an W based on ticks and steering angle
            bytes = []
            data3 = self.bus.read_byte(self.addressUno)
            bytes.append(data3)
            data4 = self.bus.read_byte(self.addressUno)
            bytes.append(data4)
            time.sleep(0.02)
            data5 = self.bus.read_byte(self.addressUno)
            bytes.append(data5)
            data6 = self.bus.read_byte(self.addressUno)
            bytes.append(data6)
            ticks = int.from_bytes(bytes, "big")
            self.ticks = ticks
            self.steeringAngle = math.radians(self.adcToAngle(ADCValue))

            self.busBusy = False

    def retrieveMessageSerial(self):
        self.ser.write(bytes("0", 'ascii'))
        adcVal = int(self.ser.readline().decode(encoding = "utf-8"))
        ticks = int(self.ser.readline().decode(encoding = "utf-8"))
        self.steeringAngle = math.radians(self.adcToAngle(adcVal))
        self.ticks = ticks

        
        

if __name__ == '__main__':
    c = Car()
