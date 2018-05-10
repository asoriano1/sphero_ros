#! /usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, ColorRGBA
import math

class Controller():
    def __init__(self):
        self.lastData = None
        rospy.Subscriber("joy", Joy, self._joyChanged)
        self.pubNav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pubSetBackLed = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.pubSetHeading = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.pubSetColor = rospy.Publisher('set_color', ColorRGBA, queue_size=1)
        self.offset = 0
        self.calibration = False
        self.red = 1
        self.green = 0
        self.blue = 0
        self.speed = 120 #default speed of movement
        
        # value = Float32()
        # value.data = 1.0
        # for i in range(0, 100):
            # self.pubSetBackLed.publish(value)

    def _joyChanged(self, data):
        self.lastData = data
        # print(data)
        
    def run(self):
        while not rospy.is_shutdown():
            msg = Twist()
            value = Float32()
            color = ColorRGBA()
            if self.lastData != None:
                if self.lastData.buttons[12] == 1:
                    while self.lastData.buttons[12] == 1:
                        pass
                    if self.red == 1:  
                        self.red = 0
                        self.green = 1
                    elif self.green == 1:
                        self.green = 0
                        self.blue = 1
                    else:
                        self.blue = 0
                        self.red = 1
                    color.r = self.red
                    color.g = self.green
                    color.b = self.blue
                    self.pubSetColor.publish(color)
                if self.lastData.buttons[11] == 1: #increasing speed
                    while self.lastData.buttons[11] == 1:
                        pass
                    self.speed += 10
                    if self.speed > 250:
                        self.speed = 250
                    print("Speed: {}".format(self.speed))
                elif self.lastData.buttons[10] == 1: #decreasing speed
                    while self.lastData.buttons[10] == 1:
                        pass
                    self.speed -= 10
                    if self.speed < 0:
                        self.speed = 0
                    print("Speed: {}".format(self.speed))
                if self.lastData.buttons[14] == 1: #FOR XBOX JOYSTICK: self.lastData.buttons[0] == 1:
                    value.data = 1.0
                    self.pubSetBackLed.publish(value)
                    self.offset += self.lastData.axes[0] * math.pi / 50.0
                    msg.angular.x = self.offset
                    msg.linear.x = 0
                    self.calibration = True
                else:
                    if self.calibration:
                        value.data = self.offset
                        self.pubSetHeading.publish(value)
                        value.data = 0
                        self.pubSetBackLed.publish(value)
                        self.calibration = False
                        self.offset = 0
                    msg.linear.x = self.lastData.axes[2] * -self.speed   #FOR XBOX JOYSTICK: self.lastData.axes[3] * -255
                    msg.linear.y = self.lastData.axes[3] * self.speed    #FOR XBOX JOYSTICK: self.lastData.axes[4] * 255
                
            self.pubNav.publish(msg)
            # self.pubSetHeading.publish(value)
            
            rospy.sleep(0.01)



if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    controller = Controller()
    controller.run()
    # rospy.spin()
