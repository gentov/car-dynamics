#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from Robot import *
class CarController:
    def __init__(self):
        self.KeyboardMessage = rospy.Subscriber('/CarKeyboard', Float64, self.keyboardCallback, queue_size=1)
        self.car = Car()

    def keyboardCallback(self, data):
        if data.data == 0:
            print("Pressed Up")
        elif data.data == 1:
            print("Pressed Down")
        elif data.data == 2:
            print("Pressed Left")
            self.car.turnLeftRelative(45)

        elif data.data == 3:
            print("Pressed Right")
            self.car.turnRightRelative(45)
        else:
            print(data)

if __name__ == '__main__':
    rospy.init_node('Car_Controller')
    rospy.sleep(.5)
    car = CarController()
    while not rospy.is_shutdown():
        pass