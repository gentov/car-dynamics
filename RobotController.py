#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from Robot import *
class CarController:
    def __init__(self):
        self.KeyboardMessage = rospy.Subscriber('/CarKeyboard', Float64, self.keyboardCallback, queue_size=1)
        self.car = Car()
        self.car.turnAbsolute(0)

    def keyboardCallback(self, data):
        if data.data == 0:
            print("Pressed Up")
            self.car.drivingMotor.setDirection(0)
            self.car.drivingMotor.turn(100)
        elif data.data == 1:
            print("Pressed Down")
            self.car.drivingMotor.setDirection(1)
            self.car.drivingMotor.turn(100)
        elif data.data == 2:
            print("Pressed Left")
            self.car.turnAbsolute(44)

        elif data.data == 3:
            print("Pressed Right")
            self.car.turnAbsolute(-44)

        elif data.data == 4:
            print("Stop")
            self.car.drivingMotor.turn(0)
        elif data.data == 5:
            self.car.turnAbsolute(0)
        elif data.data == 6:
            print("Velocity: ", self.car.drivingMotor.encoder.velocity)
            print("Ticks: ", self.car.drivingMotor.encoder.ticks)
        else:
            print(data)


if __name__ == '__main__':
    rospy.init_node('Car_Controller')
    rospy.sleep(.5)
    car = CarController()
    while not rospy.is_shutdown():
        pass