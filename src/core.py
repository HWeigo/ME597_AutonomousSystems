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
                self.isParking = 0


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
                        return
                
                    
                def imageProcessing(data):
			try:
				frame=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
			except CvBridgeError as e:
				print(e)

                        #print "Processing image."
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


                        #self.blobpub.publish(self.bridge.cv2_to_imgmsg(cropped_edges,"mono8"))
                        self.blobpub.publish(self.bridge.cv2_to_imgmsg(line_image,"bgr8"))
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
                       # self.trans_xz = [trans.x, trans.z]
                       # self.rot_z = (rot.z + self.rotzLast1 + self.rotzLast2)/3
                       # self.rotzLast2 = self.rotzLast1 
                       # self.rotzLast1 = rot.z
                        
                        #print "Fid trans x, y, z, w:  %lf, %lf, %lf, %lf \n\n" % (rot.x, rot.y, self.rot_z, rot.w)
                                
                        if self.arucoId:
                            self.isArUcoDetect = True # set the flag to True
                        if m.fiducial_id in [0,1,2]:
                            self.isParking = 1
                        
                    print(self.arucoId)
                                


		#Subscribe to topics 
		rospy.Subscriber('raspicam_node/image', Image, imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		rospy.Subscriber('distance', distance, distanceCallback)
                rospy.Subscriber("fiducial_transforms", FiducialTransformArray, fiducialNav)

		rospy.init_node('core', anonymous=True)
		self.rate = rospy.Rate(10)

        def reduce_resolution(self, frame, scale_percent):
                #calculate the 50 percent of original dimensions
                width = int(frame.shape[1] * scale_percent / 100)
                height = int(frame.shape[0] * scale_percent / 100)

                # dsize
                dsize = (width, height)

                # resize image
                frame = cv2.resize(frame, dsize)
                
                return frame 

        def detect_edges(self, frame):
                # filter for blue lane lines
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                #cv2.imshow("hsv", hsv)

                # Lab
                #lower_yellow = np.array([20, 40, 55])
                #upper_yellow = np.array([32, 255, 255])
                
                # Home Day Time
               # lower_black = np.array([180*0.03, 255*0.25, 255*0.16])
               #  upper_black = np.array([180*0.22, 255*0.75, 255*0.37])
 
                # Home Night Time
                lower_black = np.array([180*0.03, 255*0.0, 255*0.10])
                upper_black = np.array([180*0.22, 255*0.5, 255*0.24])              
                
                mask = cv2.inRange(hsv, lower_black, upper_black)
                #cv2.imshow("blue mask", mask)
                # detect edges
                edges = cv2.Canny(mask, 200, 400)

                return edges 
        
        def region_of_interest(self, edges):
                height, width = edges.shape
                mask = np.zeros_like(edges)

                # only focus bottom half of the screen
                polygon = np.array([[(0, height * 1 / 2), (width, height * 1 / 2), (width, height), (0, height), ]], np.int32)

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
                y2 = int(y1 * 1 / 2)  # make points from middle of the frame down
            
                # bound the coordinates within the frame
                x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
                x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
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
                            logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
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
            
                left_fit_average = np.average(left_fit, axis=0)
                if len(left_fit) > 0:
                    lane_lines.append(self.make_points(frame, left_fit_average))
            
                right_fit_average = np.average(right_fit, axis=0)
                if len(right_fit) > 0:
                    lane_lines.append(self.make_points(frame, right_fit_average))
            
                logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
            
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


        # Calculate robot position (delta_x, delta_z) in ground frame ( based on the tag)
        # Calculate target position (0,0,0.8) in robot frame, return in polar coordinates (rho, phi)
        def frame_transformation(self, destination, trans_xz, rot_z):
                theta = -108.9 * rot_z - 0.9179  # calculate the angle between two frames (degree)
                theta = math.radians(theta)  # convert the angle into radians
                print(theta)
                # Rotation matrix
                rotation_Rri_matrix = np.array([(math.cos(theta), -math.sin(theta)),(math.sin(theta), math.cos(theta))])
                rotation_Rir_matrix = np.array([(math.cos(theta), math.sin(theta)),(-math.sin(theta), math.cos(theta))])
                
                destination = np.array(destination)
                destination_coordinate = rotation_Rri_matrix.dot(destination) + np.array([[trans_xz[0]], [trans_xz[1]]])
                robot_position = rotation_Rir_matrix.dot(np.array([[-trans_xz[0]],[-trans_xz[1]]]))
                rho = np.sqrt(destination_coordinate[0]**2 + destination_coordinate[1]**2)
                phi = np.arctan2(destination_coordinate[1], destination_coordinate[0])
                if math.degrees(phi) < 180 and math.degrees(phi) > 0: 
                    return float(rho), math.degrees(phi) - 90, robot_position[0], robot_position[1]
                else:
                    return -float(rho), math.degrees(phi) + 90, robot_position[0], robot_position[1]

        
        def CrossHill(self, isLaneDetect):
            if self.hillNum == 0: # avoid second time speed up
                start = time.time()
                while (time.time() - start) < 0.3: # speed up for 0.3 seconds to climb the hill
                    self.leftSpeed = 0.6
                    self.rightSpeed = 0.6
                    self.publishMotors()
                self.hillNum = self.hillNum + 1
            self.leftSpeed = 0
            self.rightSpeed = 0
            self.publishMotors()

#        def PIDController(self, kp, ki, kd, curr, target):
#
#            if abs(angleDegAvg) > 35:
#                sum_angle = sum_angle + angleDegAvg * 0.01
#            else:
#                sum_angle = 0
#            steering_speed = kp * angleDegAvg + ki * sum_angle + kd * (angleDeg - angleDegLast1)/0.01  
#            if abs(steering_speed) > max_angle_deviation:
#                steering_speed = max_angle_deviation * steering_speed / abs(steering_speed) 
#            
#            return pidOutput

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
                        
        def runner(self):
                angleDegLast1 = 0
                angleDegLast2 = 0
                curr_steering_angle = 0
                last_steering_angle = 0
                sum_angle = 0

                while not rospy.is_shutdown():
                    self.laneFollow = True 
                    if self.isArUcoDetect: # check if any ArUco is detected
                        start = time.time()
                        while (time.time() - start) < 0.05: # stop for a while
                            self.leftSpeed = 0
                            self.rightSpeed = 0
                            self.publishMotors()
                        self.laneFollow = False # close lane folloer function

                        # Hill Obstacle
                        if 3 in self.arucoId: # check whether ArUco 3 is detected
                            self.CrossHill(self.numLaneDetect) # call CrossHill function
                            self.isArUcoDetect = False 
                            self.arucoId = []
                        
                    # Parking
                    if self.isParking:
                        # check the distance to the wall
                        if self.distance > 0.05:
                            garagePosition = self.arucoX[:3]
                            # if one of the ArUco is not detected, assumn it's at the edge position
                            if garagePosition[0] is -1:
                                garagePosition[0] = 0.25
                            if garagePosition[1] is -1:
                                garagePosition[1] = -0.25

                            # check whether middle ArUco is detected
                            if garagePosition[2] is not -1:
                                target = garagePosition[2]
                            else:
                                target = (garagePosition[0] + garagePosition[1])/2
                            
                            # initial forward speed
                            forward_speed = 0.07
                            # if target > 0, turn right, else turn left
                            kp_parking = 0.2
                            steering_speed = kp_parking*target  
                            if steering_speed > 0.04:
                                steering_speed = 0.04
                            self.leftSpeed = forward_speed + steering_speed
                            self.rightSpeed = forward_speed - steering_speed 
                            self.LimitSpeed(0.1, 0.22)
                            self.publishMotors()

                        else:
                            # stop the car if distance to the wal is < 0.3m
                            self.leftSpeed = 0
                            self.rightSpeed = 0
                            self.publishMotors()


                        # Lane follower & Pedestrians avoid
                    if self.numLaneDetect != 0 and self.distance > 0.3 and self.laneFollow:
                        # Calculate Road Angle
                        angleRadian = np.arctan2(self.x_offset, self.y_offset) 
                        angleDeg = angleRadian * 180.0 / math.pi  # angle (in degrees) to center vertical line
                        angleDegAvg = (angleDeg + angleDegLast1 + angleDegLast2) / 3 
                        angleDegLast2 = angleDegLast1 
                        angleDegLast1 = angleDeg 
                        #print(angleDegAvg)
                        
                        # PID Controller
                        if self.numLaneDetect == 1:
                            forward_speed = 0.08
                            max_angle_deviation = 0.1
                        if self.numLaneDetect == 2:
                            forward_speed = 0.09
                            max_angle_deviation = 0.015

                        ## ********** Config paremeter ******** 
                        kp = 0.002
                        ki = 0.001
                        kd = 0.04
                        ## ************************************
                        
                        if abs(angleDegAvg) > 35:
                            sum_angle = sum_angle + angleDegAvg * 0.01
                        else:
                            sum_angle = 0
                        steering_speed = kp * angleDegAvg + ki * sum_angle + kd * (angleDeg - angleDegLast1)/0.01  
                        if abs(steering_speed) > max_angle_deviation:
                            steering_speed = max_angle_deviation * steering_speed / abs(steering_speed) 
                        
                        self.leftSpeed = forward_speed  + steering_speed 
                        self.rightSpeed = forward_speed - steering_speed 
                        # Minimum forward_speed for the car to start moving
                        speed_upper_bound = 0.30
                        speed_lower_bound = 0.07

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
                        
	    	        self.publishMotors()                       
                    else:
                        self.rightSpeed = 0
                        self.leftSpeed = 0
                        self.publishMotors()

                    self.rate.sleep()


if __name__ == '__main__':
	autonomy().runner()
