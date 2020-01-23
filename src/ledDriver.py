#!/usr/bin/env python3

import rospy
import board
import neopixel
from autonomy.msg import leds
from time import sleep

class ledDriver:

	def __init__(self):

		def ledUpdate(data): #update LEDs
			self.pixels[0]=(data.r1,data.g1,data.b1)
			self.pixels[1]=(data.r2,data.g2,data.b2)
			self.pixels[2]=(data.r3,data.g3,data.b3)

                #create the node and subscribe
		rospy.Subscriber('leds',leds,ledUpdate)
		rospy.init_node('ledDriver',anonymous=True)
		self.rate = rospy.Rate(10)

	def runner(self):

		#initialize
		self.pixels = neopixel.NeoPixel(board.D18,3)

		while not rospy.is_shutdown():#run at 10Hz
			self.rate.sleep()

if __name__ == '__main__':
	try:
		ledDriver().runner()
	except KeyboardInterrupt:
		pass


