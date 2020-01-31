#!/usr/bin/env python

import numpy as np
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from autonomy.msg import motors, lines, distance, servos #, leds


class autonomy(object):

	def __init__(self):
		##USER PARAMETERS
		self.dummyParam = 30


		##Initiate variables
		self.leftLine = 0
		self.midLine = 0
		self.rightLine = 0
		self.distance = 0
		self.leftSpeed = 0
		self.rightSpeed = 0
		self.pan = 0
		self.tilt = 0
		self.bridge = CvBridge()

		#Setup Publishers
		self.motorPub = rospy.Publisher('motors', motors, queue_size=10)
		self.servoPub = rospy.Publisher('servos', servos, queue_size=10)
		#self.LEDpub = rospy.Publisher('leds', leds, queue_size=10)


		#Create Subscriber callbacks
		def lineCallback(data):
			self.leftLine = data.leftLine
			self.midLine = data.midLine
			self.rightLine = data.rightLine

		def distanceCallback(data):
			self.distance = data.distance


		def imageProcessing(data):
			try:
				frame=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
			except CvBridgeError as e:
				print(e)

			##Place image processing code here!
			#cv2.imwrite('test.jpg',frame)


		#Subscribe to topics
		rospy.Subscriber('raspicam_node/image',Image,imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		rospy.Subscriber('distance', distance, distanceCallback)

		rospy.init_node('core', anonymous=True)
		self.rate = rospy.Rate(10)

	def publishMotors(self):
		motorMsg = motors()
		motorMsg.leftSpeed = self.leftSpeed
		motorMsg.rightSpeed = self.rightSpeed
		rospy.loginfo(motorMsg)
		self.motorPub.publish(motorMsg)

	def publishServo(self):
		servoMsg = servos()
		servoMsg.pan = self.pan
		servoMsg.tilt = self.tilt
		rospy.loginfo(servoMsg)
		self.servoPub.publish(servoMsg)

#	def publishLED(self):
#		LEDmsg = leds()
#		LEDmsg.r1 = 255
#		LEDmsg.g1 = 0
#		LEDmsg.b1 = 0
#		LEDmsg.r2 = 0
#		LEDmsg.g2 = 255
#		LEDmsg.b2 = 0
#		LEDmsg.r3 = 0
#		LEDmsg.g3 = 0
#		LEDmsg.b3 = 255
#		rospy.loginfo(LEDmsg)
#		self.LEDpub.publish(LEDmsg)
        def forward(self, timeProcess):
                start = time.time()
                while (time.time() - start + timeProcess) < 1.5:
                    if self.distance > 0.5:
                        self.leftSpeed = 0.3
                        self.rightSpeed = 0.3
                    else:
                        runtime = time.time() - start
                        self.leftSpeed = 0
                        self.rightSpeed = 0
                        self.publishMotors()
                        return runtime + timeProcess
                    self.publishMotors()
                self.leftSpeed = 0
                self.rightSpeed = 0
                self.publishMotors()
                return -1

        
        def turnRight(self):
                start = time.time()
                while (time.time() - start) < 0.25:
                    self.leftSpeed = 0.75
                    self.rightSpeed = -0.75
                    self.publishMotors()
                self.leftSpeed = 0
                self.rightSpeed = 0
                self.publishMotors()
        
        def turnLeft(self):
                start = time.time()
                while (time.time() - start) < 0.25:
                    self.leftSpeed = -0.7
                    self.rightSpeed = 0.7
                    self.publishMotors()
                self.leftSpeed = 0
                self.rightSpeed = 0
                self.publishMotors()

        def stop(self):
                start = time.time()
                while (time.time() - start) < 0.3:
                    self.leftSpeed = 0
                    self.rightSpeed = 0
                    self.publishMotors()
                   
        def checkObstacle(self):

                leftDistance = 0
                rightDistance = 0

                #check left side
                self.pan = 0.6
                self.publishServo()
                time.sleep(0.5)
                for i in range(5):
                    leftDistance += self.distance
                    time.sleep(0.1)
                
                self.pan = 0
                self.publishServo()               
                time.sleep(0.3)

                #check right side
                self.pan = -0.6
                self.publishServo()
                time.sleep(0.5)
                for i in range(5):
                    rightDistance += self.distance
                    time.sleep(0.1)
                
                self.pan = 0
                self.publishServo()                
                
                if leftDistance > rightDistance:
                    return 1
                else:
                    return -1


        def runner(self):
                while not rospy.is_shutdown():
                    
                    while self.distance > 0.5:
                        self.leftSpeed = 0.3
                        self.rightSpeed = 0.3
                        self.publishMotors()
                    self.stop()

                    if self.checkObstacle() is 1:
                        self.turnLeft()
                    else:
                        self.turnRight()

			##Leave these lines at the end
		    self.publishMotors()
		    self.publishServo()
#			self.publishLED()
		    self.rate.sleep()


if __name__ == '__main__':
	autonomy().runner()
