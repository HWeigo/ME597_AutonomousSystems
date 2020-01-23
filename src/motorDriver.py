#!/usr/bin/env python

import rospy
from gpiozero import LED, PWMLED
from autonomy.msg import motors
from time import sleep

class motorDriver:

        def __init__(self):

                #set pins
                self.dataPin = 16
                self.clkPin = 20
                self.latchPin = 21
                self.RLpwm = 5
                self.RRpwm = 6
                self.FLpwm = 13
                self.FRpwm = 19

                def motorUpdate(data):#updata motor speeds and directions
                        if data.leftSpeed > 0:
                                aL = 0b1
                                bL = 0b0
                        else:
                                aL = 0b0
                                bL = 0b1

                        if data.rightSpeed > 0:
                                aR = 0b1
                                bR = 0b0
                        else:
                                aR = 0b0
                                bR = 0b1

                        bit0 = (aR << 7)
                        bit1 = (bR << 6)
                        bit2 = (bL << 5)
                        bit3 = (aL << 4)
                        bit4 = (aR << 3)
                        bit5 = (aL << 2)
                        bit6 = (bR << 1)
                        bit7 = bL
                        byte = bit0 | bit1 | bit2 | bit3 | bit4 | bit5 | bit6 | bit7

                        self.latch.off()
                        for x in range(8):
                                self.dat.value = (byte >> x) & 1
                                self.clk.on()
                                sleep(1/1000000.0)
                                self.clk.off()
                                sleep(1/1000000.0)
                        self.latch.on()

                        data.rightSpeed = abs(data.rightSpeed)
                        data.leftSpeed = abs(data.leftSpeed)
                        if data.rightSpeed <0.1:
                                data.rightSpeed = 0
                        if data.leftSpeed <0.1:
                                data.leftSpeed = 0

                        #change speed
                        self.mot1.value = data.leftSpeed
                        self.mot2.value = data.rightSpeed
                        self.mot3.value = data.leftSpeed
                        self.mot4.value = data.rightSpeed

                #create the node and subscribe
                rospy.Subscriber('motors',motors,motorUpdate)
                rospy.init_node('motorDriver',anonymous=True)
                self.rate = rospy.Rate(10)

        def runner(self):

                #initialize outputs
                self.dat = LED(self.dataPin)
                self.clk = LED(self.clkPin)
                self.latch = LED(self.latchPin)
                self.mot1 = PWMLED(self.RLpwm)
                self.mot2 = PWMLED(self.RRpwm)
                self.mot3 = PWMLED(self.FLpwm)
                self.mot4 = PWMLED(self.FRpwm)

                while not rospy.is_shutdown():#run at 10Hz
                        self.rate.sleep()

if __name__ == '__main__':
        try:
                motorDriver().runner()
        except KeyboardInterrupt:
                pass

