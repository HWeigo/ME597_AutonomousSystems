#!/usr/bin/env python

import rospy
from gpiozero import DigitalInputDevice
from autonomy.msg import lines

class distanceDrive:

        def __init__(self):

                #set default
                self.left = DigitalInputDevice(17)
                self.mid = DigitalInputDevice(27)
                self.right = DigitalInputDevice(22)

                #ROS publisher
                self.linePub = rospy.Publisher('lines', lines, queue_size=10)

                rospy.init_node('lines', anonymous=True)

                #this rate will drive the serial read rate
                self.rate=rospy.Rate(20)

        #assemble messages and publish
        def publishLines(self, left,mid,right):
                msg = lines()
                msg.leftLine = left
                msg.midLine = mid
                msg.rightLine = right
                rospy.loginfo(msg)
                self.linePub.publish(msg)

    #as named...
        def doTheThings(self):
                while not rospy.is_shutdown():
                        #read from the sensor
                        self.publishLines(self.left.value,self.mid.value,self.right.value)
                        self.rate.sleep()

if __name__ == '__main__':
        try:
                s = distanceDrive().doTheThings()
        except KeyboardInterrupt:
                pass
