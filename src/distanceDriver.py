#!/usr/bin/env python

import rospy
from gpiozero import DistanceSensor
from autonomy.msg import distance

class distanceDrive:

        def __init__(self):

                #set default
                self.dist = 0
                self.sensor = DistanceSensor(23,26)

                #ROS publisher
                self.distPub = rospy.Publisher('distance', distance, queue_size=10)

                rospy.init_node('distance', anonymous=True)

                #this rate will drive the serial read rate
                self.rate=rospy.Rate(10)

        #assemble messages and publish
        def publishDist(self, dist):
		msg = distance()
                msg.distance = dist
                rospy.loginfo(msg)
                self.distPub.publish(msg)

    #as named...
        def doTheThings(self):
                while not rospy.is_shutdown():
                        #read from the sensor
                        dist = self.sensor.distance
                        self.publishDist(dist)
                        self.rate.sleep()

if __name__ == '__main__':
        try:
                s = distanceDrive().doTheThings()
        except KeyboardInterrupt:
                pass
