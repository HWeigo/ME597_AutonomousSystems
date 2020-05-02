#!/usr/bin/env python

import numpy as np
import rospy
import math
import logging
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
                self.x_offset = 0
                self.y_offset = 1
                self.laneFollow = True
                self.arucoId = []
                self.numLaneDetect = 0
                self.isArUcoDetect = False 
                self.hillNum = 0
                self.reverseNum = 0
                self.isParking = 0
                self.isReverse = 0
                self.isHill = 0
                self.isTunnel = 0
                self.isRedLight = 0
                self.isIntersectionLeft = 0
                self.isIntersectionRight = 0
                self.timeDetectReverse = 0
                self.leftSlope = 0
                self.rightSlope = 0

                #Setup Publishers
		self.motorPub = rospy.Publisher('motors', motors, queue_size=10)
		self.servoPub = rospy.Publisher('servos', servos, queue_size=10)
	        self.blobpub = rospy.Publisher('imageProc',Image, queue_size=10)


		#Create Subscriber callbacks
		def lineCallback(data):
			self.leftLine = data.leftLine
			self.midLine = data.midLine
			self.rightLine = data.rightLine

		def distanceCallback(data):
			self.distance = data.distance
                        return
                
                    
                def imageProcessing(data):
			try:
				frame=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
			except CvBridgeError as e:
				print(e)

                        
                        ############# Lane Detection ################
                        frame = self.reduce_resolution(frame, 18)        
                        edges = self.detect_edges(frame)
                        cropped_edges = self.region_of_interest(edges)

                        height, width = edges.shape
                        line_segments = self.detect_line_segments(cropped_edges)
                        #print(line_segments)
                        lane_lines = self.average_slope_intercept(frame, line_segments)
                        #print(lane_lines);

                        # Calculate mid lane
                        self.numLaneDetect = 0
                        x_offset = 0
                        if len(lane_lines) == 2:
                            self.numLaneDetect = 2
                            _, _, left_x2, _ = lane_lines[0][0]
                            _, _, right_x2, _ = lane_lines[1][0]
                            mid = int(width / 2)
                            x_offset = (left_x2 + right_x2) / 2 - mid
                        if len(lane_lines) == 1:
                            self.numLaneDetect = 1
                            x1, _, x2, _ = lane_lines[0][0]
                            x_offset = x2 - x1

                        ##### Red Light Detection (Blob Detect) ####
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        lower_red = np.array([180*0.04, 255*0.0, 255*0.0])
                        upper_red = np.array([180*0.95, 255*1.0, 255*1.0])        
                        
                        # Mask other color except RED
                        mask = cv2.inRange(hsv, lower_red, upper_red)
                        # Setup SimpleBlobDetector parameters.
                        params = cv2.SimpleBlobDetector_Params()
                        # Change thresholds
                        params.minThreshold = 10
                        params.maxThreshold = 200
                        # Filter by Area.
                        params.filterByArea = True
                        params.minArea = 250
                        # Filter by Circularity
                        params.filterByCircularity = True
                        params.minCircularity = 0.1
                        params.maxCircularity = 0.9
                        # Filter by Convexity
                        params.filterByConvexity = True
                        params.minConvexity = 0.2
                        # Filter by Inertia
                        params.filterByInertia = True
                        params.minInertiaRatio = 0.01
                        # Create a detector with the parameters
                        ver = (cv2.__version__).split('.')
                        if int(ver[0]) < 3 :
                            detector = cv2.SimpleBlobDetector(params)
                        else : 
                            detector = cv2.SimpleBlobDetector_create(params)
                        # Detect blobs.
                        keypoints = detector.detect(mask)
                        # Check whether there is red light in front
                        if keypoints:
                            self.isRedLight = 1
                        else:
                            self.isRedLight = 0
                        
                        # Display lines
                        green =(0, 255, 0)
                        red = (0, 0, 255)
                        line_width=2
                        line_image = np.zeros_like(frame)
                        if lane_lines is not None:
                            for line in lane_lines:
                                for x1, y1, x2, y2 in line:
                                    cv2.line(line_image, (x1, y1), (x2, y2), green, line_width)
                            cv2.line(line_image, (width/2, height), (width/2 + x_offset, height/2), red, line_width)

                        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

                        # Display mask image
                        #self.blobpub.publish(self.bridge.cv2_to_imgmsg(cropped_edges,"mono8"))
                        # Display lane detection result
                        self.blobpub.publish(self.bridge.cv2_to_imgmsg(line_image,"bgr8"))
                        # Display blob detection result 
                        #self.blobpub.publish(self.bridge.cv2_to_imgmsg(cv2.drawKeypoints(mask, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS),"bgr8"))
                        
                        self.x_offset = x_offset 
                        self.y_offset = int(height / 2)

                
                self.rotzLast1 = 0
                self.rotzLast2 = 0
		def fiducialNav(data):
#                    print "FiducialNav callback"
                    self.isArUcoDetect = False
                    self.arucoId = [] # Store all the ids
                    self.arucoX = [-1] * 9 # Store each Aruco id's x coordinate
		    for m in data.transforms:
		        self.arucoId.append(m.fiducial_id) # record evry ArUco id appears
		        trans = m.transform.translation # calculate position
                        rot = m.transform.rotation # calculate rotation
                        self.arucoX[m.fiducial_id] = trans.x
                        
                        # 0<x<0.25: at the right side, -0.25<x<0: at the left side
                        print "Fid trans x, y, z:  %d, %lf, %lf, %lf" % (m.fiducial_id, trans.x, trans.y, trans.z) 
   
                        if self.arucoId:
                            self.isArUcoDetect = True 
                        # Check for garage obstacle
                        if m.fiducial_id in [0,1,2]:
                            self.isParking = 1
                        # Check for U turn obstacle
                        if m.fiducial_id is 6:
                            # Store the distance
                            self.isReverse = trans.z
                        # Check for tunnel obstacle
                        if m.fiducial_id is 4 or m.fiducial_id is 5:
                            self.isTunnel = 1
                        # Check for hill obstacle
                        if m.fiducial_id is 3:
                            self.isHill = trans.z # store distance to marker 3
                        # Nevigate at intersection
                        if m.fiducial_id is 7:
                            self.isIntersectionRight = 1
                        if m.fiducial_id is 8:
                            self.isIntersectionLeft = 1


		#Subscribe to topics 
		rospy.Subscriber('raspicam_node/image', Image, imageProcessing)
		rospy.Subscriber('distance', distance, distanceCallback)
                rospy.Subscriber("fiducial_transforms", FiducialTransformArray, fiducialNav)

		rospy.init_node('core', anonymous=True)
		self.rate = rospy.Rate(20)
        

        # Reduce resolution to decrease processing time
        def reduce_resolution(self, frame, scale_percent):
                # Calculate the scale_percent percent of original dimensions
                width = int(frame.shape[1] * scale_percent / 100)
                height = int(frame.shape[0] * scale_percent / 100)

                dsize = (width, height)

                # Resize image
                frame = cv2.resize(frame, dsize)
                
                return frame 

        def detect_edges(self, frame):
                # filter for blue lane lines
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
                # camera 5, 40fps 
                lower_black = np.array([180*0.05, 255*0.0, 255*0.0])
                upper_black = np.array([180*0.55, 255*0.5, 255*0.238])        
                
                #camera 6 ,60fps
                #lower_black = np.array([180*0.05, 255*0.0, 255*0.02])
                #upper_black = np.array([180*0.55, 255*0.5, 255*0.13])        
                
                mask = cv2.inRange(hsv, lower_black, upper_black)
                # Detect edges
                edges = cv2.Canny(mask, 200, 400)

                return edges 
        
        def region_of_interest(self, edges):
                height, width = edges.shape
                mask = np.zeros_like(edges)

               # # only focus bottom 3/5 of the screen
               # polygon = np.array([[(0, height * 1 /2), (width, height * 1 / 2), (width, height), (0, height), ]], np.int32)

               # cv2.fillPoly(mask, polygon, 255)
                polygon = np.array([[(0, height * 9 /20), (width, height * 9 / 20), (width, height), (0, height), ]], np.int32)

                cv2.fillPoly(mask, polygon, 255)

                cropped_edges = cv2.bitwise_and(edges, mask)
                return cropped_edges


        def detect_line_segments(self, cropped_edges):
                # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
                rho = 1  # distance precision in pixel, i.e. 1 pixel
                angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
                min_threshold = 15  # minimal of votes
                line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                np.array([]), minLineLength=20, maxLineGap=4)
        
                return line_segments	

        def make_points(self, frame, line):
                height, width, _ = frame.shape
                slope, intercept = line
                y1 = height  # bottom of the frame
                y2 = int(y1 * 9 / 20)  # make points from middle of the frame down
               
                if slope is 0:
                    x1 = 0
                    x2 = 0
                else:
                    x1 = (y1 - intercept) / slope 
                    x2 = (y2 - intercept) / slope 
                
                if x1 == float("inf"):
                    x1 = int(2*width) 
                elif x1 == float("-inf"):
                    x1 = int(-width) 
                else:
                    x1 = max(-width, min(2 * width, int(x1)))

                if x2 == float("inf"):
                    x2 = int(2*width) 
                elif x2 == float("-inf"):
                    x2 = int(-width) 
                else:
                    x2 = max(-width, min(2 * width, int(x2)))

                return [[x1, y1, x2, y2]]


        def average_slope_intercept(self, frame, line_segments):
                """
                This function combines line segments into one or two lane lines
                If all line slopes are < 0: then we only have detected left lane
                If all line slopes are > 0: then we only have detected right lane
                """
                lane_lines = []
                if line_segments is None:
                    logging.info('No line_segment segments detected')
                    return lane_lines
            
                height, width, _ = frame.shape
                left_fit = []
                right_fit = []
            
                boundary = 1/2
                left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
                right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen
            
                for line_segment in line_segments:
                    for x1, y1, x2, y2 in line_segment:
                        if x1 == x2:
                            #logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                            continue
                        fit = np.polyfit((x1, x2), (y1, y2), 1)
                        slope = fit[0]
                        intercept = fit[1]
                        if slope < 0:
                            if x1 < left_region_boundary and x2 < left_region_boundary:
                                left_fit.append((slope, intercept))
                        else:
                            if x1 > right_region_boundary and x2 > right_region_boundary:
                                right_fit.append((slope, intercept))
                
                self.leftSlope = 0
                left_fit_average = np.average(left_fit, axis=0)
                if len(left_fit) > 0:
                    lane = self.make_points(frame, left_fit_average)
                    lane_lines.append(lane)
                    x1, y1, x2, y2 = lane[0]
                    self.leftSlope =  np.arctan2((x2-x1), (height/2)) * 180.0 / math.pi 
                
                self.rightSlope = 0
                right_fit_average = np.average(right_fit, axis=0)
                if len(right_fit) > 0:
                    lane = self.make_points(frame, right_fit_average)
                    lane_lines.append(lane)
                    x1, y1, x2, y2 = lane[0]
                    self.rightSlope = np.arctan2((x2-x1), (height/2)) * 180.0 / math.pi 

                #logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
            
                return lane_lines


        def publishMotors(self):
		motorMsg = motors()
		motorMsg.leftSpeed = self.leftSpeed
		motorMsg.rightSpeed = self.rightSpeed
		#rospy.loginfo(motorMsg)
		self.motorPub.publish(motorMsg)

	def publishServo(self):
		servoMsg = servos()
		servoMsg.pan = self.pan
		servoMsg.tilt = self.tilt
		#rospy.loginfo(servoMsg)
		self.servoPub.publish(servoMsg)

        # Used to climb the hill
        def CrossHill(self, speed, t):
            start = time.time()
            while (time.time() - start) < t: # Speed up for 0.3 seconds to climb the hill
                self.leftSpeed = speed 
                self.rightSpeed = speed 
                self.publishMotors()
                self.rate.sleep()

        # Use lower bound and upper bound to limit the speed
        def LimitSpeed(self, speed_lower_bound, speed_upper_bound):

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

        # Stop the car for t seconds
        def Stop(self, t):
            start = time.time()
            while (time.time() - start) < t: # stop for a while
                self.leftSpeed = 0
                self.rightSpeed = 0
                self.publishMotors()
                self.rate.sleep()
        
        # Drive wheels at a certain spped within a certain time
        def DriveMotors(self, left_speed, right_speed, t):
            start = time.time()
            while (time.time() - start) < t: # stop for a while
                self.leftSpeed = left_speed 
                self.rightSpeed = right_speed 
                self.publishMotors()
                self.rate.sleep()

        def runner(self):
                angleDegLast1 = 0
                angleDegLast2 = 0
                curr_steering_angle = 0
                last_steering_angle = 0
                sum_angle = 0
                stopBeforeTurn = 10
                stopAtLine = 10
                timeDetectTunnel = 0
                while not rospy.is_shutdown():
                    currTime = time.time()
                    # Allow lane follow function
                    self.laneFollow = True  

                    # Hill Obstacle
                    # Check whether whether there is marker 6 and within a certain distance
                    if self.isHill and self.isHill < 0.4:
                        # Drive the car slowly to foot of the hill
                        self.DriveMotors(0.18,0.18,1.2)
                        self.Stop(0.2)
                        # Speed up to cross the hill
                        self.CrossHill(0.5, 0.9)
                        self.Stop(0.1)
                        self.isHill = 0

                    # U Turn 
                    # Check whether whether there is marker 6 and within a certain distance
                    if self.isReverse and self.isReverse < 0.5:
                        self.Stop(0.2)
                        self.DriveMotors(0.35,-0.35,1.3)
                        self.Stop(0.2)
                        while self.leftSlope > 45 or self.rightSlope < -4.5:
                            self.leftSpeed = 0.24
                            self.rightSpeed = -0.24
                            self.publishMotors()
                            self.rate.sleep

                        self.isReverse = 0

                        
                    # Parking
                    if self.isParking:
                        # Check the distance to the wall
                        if self.distance > 0.05:
                            garagePosition = self.arucoX[:3]
                            # If one of the ArUco is not detected, assumn it's at the edge position
                            if garagePosition[0] is -1:
                                garagePosition[0] = 0.25
                            if garagePosition[1] is -1:
                                garagePosition[1] = -0.25

                            # If middle ArUco is detected, directlly head to marker 2
                            if garagePosition[2] is not -1:
                                target = garagePosition[2]
                            else:
                                target = (garagePosition[0] + garagePosition[1])/2
                            
                            # Initial forward speed
                            forward_speed = 0.07
                            # If target > 0, turn right, else turn left
                            # P controller to control direction
                            kp_parking = 0.25
                            steering_speed = kp_parking*target  
                            if steering_speed > 0.05:
                                steering_speed = 0.05
                            self.leftSpeed = forward_speed + steering_speed
                            self.rightSpeed = forward_speed - steering_speed 
                            self.LimitSpeed(0.8, 0.2)
                            self.publishMotors()

                        else:
                            # Stop the car if distance to the wal is < 0.3m
                            self.leftSpeed = 0
                            self.rightSpeed = 0
                            self.publishMotors()
                   
                    hasTurn = 0
                    if self.isTunnel:
                        #self.laneFollow = False 
                        #start =time.time()
                        #while (tilt.time() - start) < 0.1 or self.distance > 0.3
                       # if timeDetectTunnel is 0:
                       #     timeDetectTunnel = time.time()
                       #     self.Stop(1)
                        self.DriveMotors(0.2,0.2,0.6)
                        self.Stop(0.3)
                        distance2Wall = 0.4
                        startTurnning = 0
                        while self.isTunnel:
                            if self.distance < distance2Wall:
                                #self.leftSpeed = 0.23
                                #self.rightSpeed = -0.25
                                #print "turn "
                                self.DriveMotors(0.16, -0.3, 0.18)
                                startTurnning += 1
                            else:
                                #self.leftSpeed = 0.17
                                #self.rightSpeed = 0.17
                                self.DriveMotors(0.2,0.2,0.16) 
                            #self.publishMotors()
                            
                            self.Stop(0.2)
                            if startTurnning > 2:
                                distance2Wall = 0.5
                                if self.distance > 0.3:
                                    self.isTunnel = 0
                        self.DriveMotors(0.2,0.2,1.6)
                   
                   
                   # if self.leftSlope > 50 and self.rightSlope < -50:
                   #     self.Stop(0.3)
                   #     if self.leftSlope > 50 and self.rightSlope < -50:
                    if self.isIntersectionLeft or self.isIntersectionRight:
                        self.DriveMotors(0.2,0.2,0.6)
                        self.Stop(1.2)
                        self.DriveMotors(0.2,0.2,0.9)
                        self.Stop(0.1)
                        if self.isIntersectionLeft:
                            self.DriveMotors(0.3, -0.3, 1.1)
                        if self.isIntersectionRight:
                            self.DriveMotors(-0.3, 0.3, 1.1)
                        self.isIntersectionRight = 0
                        self.isIntersectionLeft = 0

                   # currTime = time.time()
                   # if finishStop is 1:
                   #     timeDetectStopLane = currTime 
                   #     finishStop = 0
                   # if isStopLane 
                   # # Lane follower & Pedestrians avoid

                    if self.isRedLight:
                        self.laneFollow = False
                        self.leftSpeed = 0
                        self.rightSpeed = 0
                        self.publishMotors()


                    if self.numLaneDetect != 0 and self.distance > 0.2 and self.laneFollow:
                        # Calculate Road Angle
                        angleRadian = np.arctan2(self.x_offset, self.y_offset) 
                        angleDeg = angleRadian * 180.0 / math.pi  # angle (in degrees) to center vertical line
                        angleDegAvg = angleDeg*1/6 + angleDegLast1*2/6 + angleDegLast2*3/6 
                        angleDegLast2 = angleDegLast1 
                        angleDegLast1 = angleDeg 
                        print(angleDegAvg)
      
                        print "left: %f, right: %f" % (self.leftSlope, self.rightSlope)
                        # PID Controller
 
                        ## ********** Config paremeter ******** 
                        kp = 0.002
                        ki = 0.002
                        kd = 0.0008
                        ## ************************************                      
                        
                        if self.numLaneDetect == 1:
                            if self.leftSlope > 50 or self.rightSlope < -50:
                                if stopBeforeTurn > 3:
                                    start =time.time()
                                    while (time.time() - start) < 0.1: # stop for a while
                                        self.leftSpeed = 0
                                        self.rightSpeed = 0
                                        self.publishMotors()
                                        self.rate.sleep()
                                stopBeforeTurn = 0
                                kp = 0.0022
                                forward_speed = -0.02
                                max_angle_deviation = 0.25
                            else:
                                forward_speed = 0.12
                                max_angle_deviation = 0.05
                                stopBeforeTurn += 1
                        elif self.numLaneDetect == 2:
                            forward_speed = 0.12
                            max_angle_deviation = 0.03


                        
                        if abs(angleDegAvg) > 55:
                            sum_angle = sum_angle + angleDegAvg * 0.01
                            sum_angle = min(sum_angle, 20)
                        else:
                            sum_angle = 0
                        #print(sum_angle)
                        steering_speed = kp * angleDegAvg + ki * sum_angle + kd * (angleDeg - angleDegLast1)
                        if abs(steering_speed) > max_angle_deviation:
                            steering_speed = max_angle_deviation * steering_speed / abs(steering_speed) 
                        
                        self.leftSpeed = forward_speed  + steering_speed 
                        self.rightSpeed = forward_speed - steering_speed 
                        # Minimum forward_speed for the car to start moving
                        self.LimitSpeed(0.03,0.31)
                        
	    	        self.publishMotors()

                    elif self.distance < 0.35 and self.laneFollow:
                        self.rightSpeed = 0.0
                        self.leftSpeed = 0.0
                        self.publishMotors()
                    elif self.laneFollow:
                        self.rightSpeed = 0.15
                        self.leftSpeed = 0.15
                        self.publishMotors()

                    self.rate.sleep()


if __name__ == '__main__':
	autonomy().runner()
