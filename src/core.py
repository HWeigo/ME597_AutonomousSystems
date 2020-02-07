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
	        self.blobpub = rospy.Publisher('imageProc',Image, queue_size=10)
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
                        
                        # Setup SimpleBlobDetector parameters.
                        params = cv2.SimpleBlobDetector_Params()

                        # Change thresholds
                        params.minThreshold = 10
                        params.maxThreshold = 200


                        # Filter by Area.
                        params.filterByArea = True
                        params.minArea = 1500

                        # Filter by Circularity
                        params.filterByCircularity = True
                        params.minCircularity = 0.1

                        # Filter by Convexity
                        params.filterByConvexity = True
                        params.minConvexity = 0.87
                            
                        # Filter by Inertia
                        params.filterByInertia = True
                        params.minInertiaRatio = 0.01

                        # Create a detector with the parameters
                        ver = (cv2.__version__).split('.')
                        if int(ver[0]) < 3 :
                            detector = cv2.SimpleBlobDetector(params)
                        else : 
                            detector = cv2.SimpleBlobDetector_create(params)
                        # Set up the detector with default parameters.
                        # detector = cv2.SimpleBlobDetector()
                          
                        # Detect blobs.
                        keypoints = detector.detect(frame)
                           
                        self.blobpub.publish(self.bridge.cv2_to_imgmsg(cv2.drawKeypoints(frame, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS),"bgr8"))


		#Subscribe to topics
		rospy.Subscriber('raspicam_node/image_rect_color',Image,imageProcessing)
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
                   
        # discrete obstacle avoidance
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
                distances = [0,0,0]
                panAngle_l2r = [-0.6, 0, 0.6]
                panAngle_r2l = [0.6, 0, -0.6]
                constSpeed = 0.17
                stepSpeed = constSpeed * 2
                forwardBonus = 0.05
                samplingTime = 0.15
                while not rospy.is_shutdown():

                  # # swapping sensor from left to right 
                  #  for i in range(len(panAngle_l2r)):
                  #      self.pan = panAngle_l2r[i]
                  #      self.publishServo()
                  #      time.sleep(samplingTime)

                  #      # measure distance and save the value to corrspponding position
                  #      distances[i] = self.distance
                  #      distances[1] += forwardBonus
                  #      
                  #      # find the direction with largest distacnes
                  #      nextstep = distances.index(max(distances)) - 1 
                  #      
                  #      # calculate left and right speed
                  #      self.leftSpeed = constSpeed + nextstep*stepSpeed
                  #      self.rightSpeed = constSpeed - nextstep*stepSpeed
                  #      self.publishMotors()
                  #  
                  #  # swapping sensor from right to left
                  #  for i in range(len(panAngle_r2l)):
                  #      self.pan = panAngle_r2l[i]
                  #      self.publishServo()
                  #      time.sleep(samplingTime)
                  #      distances[2-i] = self.distance
                  #      distances[1] += forwardBonus
                  #      nextstep = distances.index(max(distances)) - 1
                  #      self.leftSpeed = constSpeed + nextstep*stepSpeed
                  #      self.rightSpeed = constSpeed - nextstep*stepSpeed
                  #      self.publishMotors()


		    ##Leave these lines at the end
		    self.publishMotors()
		    self.publishServo()
#		    self.publishLED()
		    self.rate.sleep()


if __name__ == '__main__':
	autonomy().runner()
