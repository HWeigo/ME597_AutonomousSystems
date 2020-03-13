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
                self.numLaneDetect = 0

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
                    id = -1
                    #print "FiducialNav callback"
                    self.isArUcoDetect = False

		    for m in data.transforms:
			id = m.fiducial_id
			trans = m.transform.translation
                        rot = m.transform.rotation
                        print "Fid trans x, y, z:  %d, %lf, %lf, %lf" % (id, trans.x, trans.y, trans.z)
                        self.trans_xz = [trans.x, trans.z]
                        self.rot_z = (rot.z + self.rotzLast1 + self.rotzLast2)/3
                        self.rotzLast2 = self.rotzLast1 
                        self.rotzLast1 = rot.z
                        
                        print "Fid trans x, y, z, w:  %lf, %lf, %lf, %lf \n\n" % (rot.x, rot.y, self.rot_z, rot.w)
                                
                        if id is 2:
                                self.isArUcoDetect = True
                                


		#Subscribe to topics 
		rospy.Subscriber('raspicam_node/image', Image, imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		rospy.Subscriber('distance', distance, distanceCallback)
		#rospy.Subscriber("fiducial_transforms", FiducialTransformArray, fiducialNav)

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
                
                # Home
                lower_yellow = np.array([20, 140, 175])
                upper_yellow = np.array([33, 237, 238])
               
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
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

        def runner(self):
                angleDegLast1 = 0
                angleDegLast2 = 0
                curr_steering_angle = 0
                last_steering_angle = 0
                while not rospy.is_shutdown():
                    if self.numLaneDetect != 0:
                        angleRadian = np.arctan2(self.x_offset, self.y_offset) 
                        angleDeg = angleRadian * 180.0 / math.pi  # angle (in degrees) to center vertical line
                        angleDegAvg = (angleDeg + angleDegLast1 + angleDegLast2) / 3 
                        angleDegLast2 = angleDegLast1 
                        angleDegLast1 = angleDeg 
                        #print(angleDegAvg)
                        
                        # Controller 1
                        if self.numLaneDetect == 1:
                            forward_speed = 0.1 
                            max_angle_deviation = 0.1 
                        if self.numLaneDetect == 2:
                            forward_speed = 0.09
                            max_angle_deviation = 0.04

                        #angle_deviation = curr_steering_angle - last_steering_speed
                        #if abs(angle_deviation) > max_angle_deviation:
                        #    last_steering_speed 

                        ## ********** Config paremeter ******** 
                        kp = 0.03
                        ki = 0.0
                        kd = 0.0
                        ## ************************************
                        
                        steering_speed = kp * angleDegAvg 
                        if abs(steering_speed) > max_angle_deviation:
                            steering_speed = max_angle_deviation * steering_speed / abs(steering_speed) 
                        
                       # # Controller 2
                       # max_angle_deviation_two_lines = 0.1
                       # max_angle_deviation_one_lane = 0.02
                       # if self.numLaneDetect == 2 :
                       #     # if both lane lines detected, then we can deviate more
                       #     max_angle_deviation = max_angle_deviation_two_lines
                       #     forward_speed = 0.1 
                       # else :
                       #     # if only one lane detected, don't deviate too much
                       #     max_angle_deviation = max_angle_deviation_one_lane
                       #     forward_speed = 0.12

                       # new_steering_angle = angleDegAvg 
                       # print("ang" + str(angleDeg))
                       # angle_deviation = new_steering_angle - curr_steering_angle
                       # if abs(angle_deviation) > max_angle_deviation:
                       #     stabilized_steering_angle = curr_steering_angle  + max_angle_deviation * angle_deviation / abs(angle_deviation)
                       # else:
                       #     stabilized_steering_angle = new_steering_angle 
                       # curr_steering_angle = new_steering_angle 
                       #     
                       # ## ********** Config paremeter ******** 
                       # kp = 0.02
                       # ki = 0.0
                       # kd = 0.02
                       # max_steering_speed = 0.15
                       # ## ************************************
                       # 
                       # print(stabilized_steering_angle)
                       # steering_speed = kp * stabilized_steering_angle + kd * (stabilized_steering_angle-last_steering_angle)/100 
                       # if abs(steering_speed) > max_steering_speed:
                       #     steering_speed = max_steering_speed
                       # last_steering_angle = stabilized_steering_angle 
                       # #print(steering_speed)                        
                        
                        
                        
                        self.leftSpeed = forward_speed  + steering_speed 
                        self.rightSpeed = forward_speed - steering_speed 
                        print(steering_speed)
                        # Minimum forward_speed for the car to start moving
                        speed_upper_bound = 0.30
                        speed_lower_bound = 0.13

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

                       # errorCurr = angleDeg 
                       # errorSum += errorCurr * 0.01
    
                       # integralBound = 0.3
                       # if errorSum > integralBound:
                       #     errosrSum = integralBound
                       # if errorSum < (-1 * integralBound):
                       #     errorSum = -1 * integralBound
                       # forward_speed  = kp * errorCurr + ki * errorSum + kd * (errorCurr - errorLast) / 0.01 # Calculate PID output
                       # errorLast = errorCurr                                       
                       #                        
                       # # ******** Restrict output ***********
                       # forward_upper_bound = 0.05
                       # if forward_speed > forward_upper_bound:
                       #     forward_speed  = forward_upper_bound 
                       # if forward_speed < -forward_upper_bound:
                       #     forward_speed = -forward_upper_bound 
                       # #*************************************

                       # if phi_average > 25:
                       #     phi_average = 25
                       # if phi_average < -25:
                       #     phi_average = -25
                       # ks = 0.04
                       # kc = 0.15
                       # kxp = 2.5
                       # kxi = 8.0
                       # self.leftSpeed = forward_speed 
                       # self.rightSpeed = forward_speed 
                       # 
                       # # Check whether it's going to loss vision
                       # if alpha_average > 20 or alpha_average < -20:
                       #     # If alpha is too large, turn a little bit to make sure tag can still be captured by camera
                       #     steering_speed = kc * alpha_average
                       # else:
                       #     # Use PI controller to make robot approach y axis
                       #     delta_x_sum += delta_x * 0.01
                       #     steering_speed = kxp * delta_x + kxi * delta_x_sum 
                       #     
                       #     # ******** Restrict output ***********
                       #     steering_upper_bound = 0.18
                       #     if steering_speed > steering_upper_bound:
                       #         steering_speed =steering_upper_bound 
                       #     if steering_speed < -steering_upper_bound:
                       #         steering_speed = -steering_upper_bound 
                       #     #*************************************

                       # print "forward speed: %f\n" % forward_speed
                       # print "steering speed: %f\n" % steering_speed
                       # 


                       # leftLast = self.leftSpeed 
                       # rightLast = self.rightSpeed 
                    else:
                        self.rightSpeed = 0
                        self.leftSpeed = 0
                        self.publishMotors()
                    
		    self.rate.sleep()


if __name__ == '__main__':
	autonomy().runner()
