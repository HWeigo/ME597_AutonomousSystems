#!/usr/bin/env python

import rospy
from gpiozero import Servo
from autonomy.msg import servos

class servoDrive:

        def __init__(self):

                #set default
                self.pan = Servo(4)
                self.tilt = Servo(25)


		def servoUpdate(data):#update servos
			self.pan.value = data.pan
			self.tilt.value = data.tilt

		#create the node and subscribe
		rospy.Subscriber('servos',servos,servoUpdate)
		rospy.init_node('servos',anonymous=True)
		self.rate = rospy.Rate(10)

    #as named...
        def doTheThings(self):
                while not rospy.is_shutdown():
                        #read from the sensor
                        self.rate.sleep()

if __name__ == '__main__':
        try:
                s = servoDrive().doTheThings()
        except KeyboardInterrupt:
                pass
