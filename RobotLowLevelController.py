#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from Robot import *
class CarController:
    def __init__(self):
        """
        Thoughts on car low level controller (Will rename this class to CarLowLevelController):
            1) We can have another Subscriber for receiving commands from the high level controller
            2) The high level controller will return to the low level controller:
                i) an already limited velocity
                ii) an already limited steering angle

        Thoughts on car high level controller:
            1) Just like this class, we can have another class, but for doing Input/Output control (CarHighLevelController)
            2) An attribute of this class will be a trajectory data structure
            3) the trajectory data structure will have the desired x,y, and theta at instances in time
            4) the trajectory data structure (a dictionary, perhaps) will be populated in a generateTrajectory method
            5) The generate trajectory method will go through some trajectory, finding xPos, yPos, and theta at time t,
               and populate the dictionary (or other data structure) like this: t: [xDes, yDes, theta]. This way we can
               easily grab the three desired parameters at any time
            6) At this point, our trajectory is set! WOOOHOOOO
            ------------------------------------------------------------------------------------------------------------
            Now, we have to determine the control inputs that we send to the low level controller
            7) We have to write out our T matrix, which transforms the robots back axle point to the tracked point, at the
               front of the wheels, using b (the measured length of the car)
            8) Now, we set our two gain values, which we can tweak, K1 and K2
            9) We find the coordinates of our tracked point using the current x,y, and theta of the robot (at time t)
            10) We find the DESIRED coordinates of our tracked point by using the desired x,y and theta of the robot (at time t)
            11) We compute our control law**:
                    u1 = k1*(y1d - y1)
                    u2 = k2*(y2d - y1)
            12) We invert the T matrix and multiply it by [u1 u2] --> this will give us control inputs v and w
            13) We limit v and w to the physical limitations of the robot
                i) we use w to calculate the steering angle and send it to the low-level controller
                ii) we send v directly to the low level controller
            14) We use v and w to estimate the new current x,y, and theta of the robot


            **(this is where your code and I differ, but I think the end result is the same, because of how we generate trajectory)
        """
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