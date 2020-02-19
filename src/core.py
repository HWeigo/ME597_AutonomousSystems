#!/usr/bin/env python

import numpy as np
import rospy
import math
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
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
                self.trans_xz = [0,0]
                self.rot_z = 0
                self.isArUcoDetect = False

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
			#self.distance = data.distance
                        return

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
                        params.minCircularity = 0.5
                        params.maxCircularity = 0.8

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


		def fiducialNav(data):
                    id = -1
                    print "FiducialNav callback"
                    self.isArUcoDetect = False

		    for m in data.transforms:
			id = m.fiducial_id
			trans = m.transform.translation
                        rot = m.transform.rotation
                        print "Fid trans x, y, z:  %d, %lf, %lf, %lf" % (id, trans.x, trans.y, trans.z)
                        print "Fid trans x, y, z, w:  %lf, %lf, %lf, %lf \n\n" % (rot.x, rot.y, rot.z, rot.w)
                        self.trans_xz = [trans.x, trans.z]
                        self.rot_z = rot.z
                                
                        if id is 2:
                                self.isArUcoDetect = True
                                


		#Subscribe to topics
		rospy.Subscriber('raspicam_node/image_rect_color',Image,imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		rospy.Subscriber('distance', distance, distanceCallback)
		rospy.Subscriber("fiducial_transforms", FiducialTransformArray, fiducialNav)

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
		#rospy.loginfo(servoMsg)
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

        def frame_transformation(self, destination, trans_xz, rot_z):
                theta = -108.9 * rot_z - 0.9179  # calculate the angle between two frames (degree)
                theta = math.radians(theta)  # convert the angle into radians
                print(theta)
                rotation_Rri_matrix = np.array([(math.cos(theta), -math.sin(theta)),(math.sin(theta), math.cos(theta))])
                rotation_Rir_matrix = np.array([(math.cos(theta), math.sin(theta)),(-math.sin(theta), math.cos(theta))])
                #print(rotation_Rri_matrix)
                destination = np.array(destination)
                #print(destination)
                destination_coordinate = rotation_Rri_matrix.dot(destination) + np.array([[trans_xz[0]], [trans_xz[1]]])
                robot_position = rotation_Rir_matrix.dot(np.array([[-trans_xz[0]],[-trans_xz[1]]]))
                #print(destination_coordinate)
                rho = np.sqrt(destination_coordinate[0]**2 + destination_coordinate[1]**2)
                phi = np.arctan2(destination_coordinate[1], destination_coordinate[0])
                if math.degrees(phi) < 180 and math.degrees(phi) > 0: 
                    return float(rho), math.degrees(phi) - 90, robot_position[0]
                else:
                    return -float(rho), math.degrees(phi) + 90, robot_position[0]

        def runner(self):
                errorSum = 0;
	        errorLast = 0
                rhoLast1 = 0
                rhoLast2 = 0
                phiLast1 = 0
                phiLast2 = 0
                alphaLast1 = 0
                alphaLast2 = 0
#                runTime = 0
                while not rospy.is_shutdown():
                    if self.isArUcoDetect is True:
                        [rhoCurr, phiCurr, delta_x] = self.frame_transformation([[0],[-0.3]], self.trans_xz, self.rot_z)
                        alphaCurr = np.arctan2(self.trans_xz[1], self.trans_xz[0])
                        alphaCurr = math.degrees(alphaCurr) - 90

                        # print "rho: %f, phi: %f\n" % (rhoCurr, phiCurr)
                        rho_average = (rhoCurr + rhoLast1 + rhoLast2)/3
                        phi_average = (phiCurr + phiLast1 + phiLast2)/3
                        alpha_average = (alphaCurr + alphaLast1 + alphaLast2)/3

                        rhoLast2 = rhoLast1
                        rhoLast1 = rhoCurr
                        phiLast2 = phiLast1
                        phiLast1 = phiCurr
                        alphaLast2 = alphaLast1
                        alphaLast1 = alphaCurr
                        print "rho: %f,\nphi: %f,\n alpha: %f,\ndelta_x:%f" % (rho_average, phi_average, alpha_average, delta_x)
                        if delta_x < 0.15 and delta_x > -0.15:
                            kc = 0.2
                            if alpha_average > 5 or alpha_average < -5:
                                steering_speed = kc * alpha_average
                                # ******** Restrict output ***********
                                steering_upper_bound = 0.12
                                if steering_speed > steering_upper_bound:
                                    steering_speed =steering_upper_bound 
                                if steering_speed < -steering_upper_bound:
                                    steering_speed = -steering_upper_bound 
                                #*************************************
                                self.leftSpeed = -steering_speed 
                                self.rightSpeed = steering_speed 
                            else:
                                self.leftSpeed = 0
                                self.rightSpeed = 0 

                            # Minimum forward_speed for the car to start moving
                            speed_upper_bound = 0.32
                            speed_lower_bound = 0.12

                            if self.leftSpeed > 0:
                                self.leftSpeed += speed_lower_bound 
                            if self.leftSpeed < 0:
                                self.leftSpeed -= speed_lower_bound 
                            if self.leftSpeed > speed_upper_bound:
                                self.leftSpeed = speed_upper_bound
                            if self.leftSpeed < -speed_upper_bound:
                                self.leftSpeed = -speed_upper_bound

                            if self.rightSpeed > 0:
                                self.rightSpeed += speed_lower_bound 
                            if self.rightSpeed < 0:
                                self.rightSpeed -= speed_lower_bound 
                            if self.rightSpeed > speed_upper_bound:
                                self.rightSpeed = speed_upper_bound 
                            if self.rightSpeed < -speed_upper_bound:
                                self.rightSpeed = -speed_upper_bound 
                            ## ************************************ 

		    ##Le    ave these lines at the end
	    	            self.publishMotors()
		            self.publishServo()
                        else:
                            forward_speed  = 0
                            errorCurr = 0
                    
                            ## ********** Config paremeter ******** 
                            kp = 0.6
                            ki = 0.0
                            kd = 0.020
                            targetUltr = 0.195
                            ## ************************************
                            
                            errorCurr = rho_average
                            errorSum += errorCurr * 0.01
    
                            integralBound = 0.3
                            if errorSum > integralBound:
                                errosrSum = integralBound
                            if errorSum < (-1 * integralBound):
                                errorSum = -1 * integralBound
                            forward_speed  = kp * errorCurr + ki * errorSum + kd * (errorCurr - errorLast) / 0.01 # Calculate PID output
                            errorLast = errorCurr                                       
                                            
                                                   
                            # ******** Restrict output ***********
                            forward_upper_bound = 0.10
                            if forward_speed > forward_upper_bound:
                                forward_speed  = forward_upper_bound 
                            if forward_speed < -forward_upper_bound:
                                forward_speed = -forward_upper_bound 
                            #*************************************


                            if phi_average > 25:
                                phi_average = 25
                            if phi_average < -25:
                                phi_average = -25
                            ks = 0.04
                            kc = 0.3
                            if alpha_average > 18 or alpha_average < -18:
                                steering_speed = kc * alpha_average
                            else:
                                steering_speed = ks * phi_average 
                                # ******** Restrict output ***********
                                steering_upper_bound = 0.12
                                if steering_speed > steering_upper_bound:
                                    steering_speed =steering_upper_bound 
                                if steering_speed < -steering_upper_bound:
                                    steering_speed = -steering_upper_bound 
                                #*************************************

                            print "forward speed: %f\n" % forward_speed
                            print "steering speed: %f\n" % steering_speed

                            self.leftSpeed = forward_speed  - steering_speed 
                            self.rightSpeed = forward_speed + steering_speed 

                            # Minimum forward_speed for the car to start moving
                            speed_upper_bound = 0.30
                            speed_lower_bound = 0.17

                            if self.leftSpeed > 0:
                                self.leftSpeed += speed_lower_bound 
                            if self.leftSpeed < 0:
                                self.leftSpeed -= speed_lower_bound 
                            if self.leftSpeed > speed_upper_bound:
                                self.leftSpeed = speed_upper_bound
                            if self.leftSpeed < -speed_upper_bound:
                                self.leftSpeed = -speed_upper_bound

                            if self.rightSpeed > 0:
                                self.rightSpeed += speed_lower_bound 
                            if self.rightSpeed < 0:
                                self.rightSpeed -= speed_lower_bound 
                            if self.rightSpeed > speed_upper_bound:
                                self.rightSpeed = speed_upper_bound 
                            if self.rightSpeed < -speed_upper_bound:
                                self.rightSpeed = -speed_upper_bound 
                            ## ************************************ 

		    ##Le    ave these lines at the end
	    	            self.publishMotors()
		            self.publishServo()
#		    self.publishLED()
		    self.rate.sleep()


if __name__ == '__main__':
	autonomy().runner()
