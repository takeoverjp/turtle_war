#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time
import numpy as np

from abstractBot import *
from geometry_msgs.msg import Twist

class SampleBot(AbstractBot):
    def __init__(self, name):
        super(SampleBot, self).__init__(name)
        self.score_l = 0
        self.score_r = 0
        self.score_c = 0
    def callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        rmask = np.zeros(h.shape, dtype=np.uint8)
        rmask[((h < 20) | (h > 200)) & (s > 128)] = 255
        ymask = np.zeros(h.shape, dtype=np.uint8)
        ymask[((h > 10) & (h < 25)) & (s > 50) & (v > 50)] = 255
        cv2.imshow("RMask Window", rmask)
        cv2.imshow("YMask Window", ymask)
        cv2.waitKey(1)

        imgshape=rmask.shape  

        rroi_l=rmask[:,0:imgshape[1]/3] 
        rroi_r=rmask[:,2*imgshape[1]/3:imgshape[1]] 
        rroi_c=rmask[:,imgshape[1]/3:2*imgshape[1]/3] 

        self.score_l = cv2.countNonZero(rroi_l)
        self.score_r = cv2.countNonZero(rroi_r)
        self.score_c = cv2.countNonZero(rroi_c)

        yroi_l=ymask[:,0:imgshape[1]/3] 
        yroi_r=ymask[:,2*imgshape[1]/3:imgshape[1]] 
        yroi_c=ymask[:,imgshape[1]/3:2*imgshape[1]/3] 

        self.score_l = self.score_l + cv2.countNonZero(yroi_l) * 20
        self.score_r = self.score_r + cv2.countNonZero(yroi_r) * 20
        self.score_c = self.score_c + cv2.countNonZero(yroi_c) * 20

    def strategy(self):
        r = rospy.Rate(100)

        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            if self.center_bumper or self.left_bumper or self.right_bumper:
                control_speed = -0.5
                control_turn = 0
            
            if self.score_l == 0 and self.score_c == 0 and self.score_r == 0:
                control_speed = 0
                control_turn = 3
                print "KEEP"
            elif self.score_l > self.score_r:
                if self.score_l > self.score_c:
                    control_speed = 0
                    control_turn = 3
                    print "left : " + str(self.score_l)
                else:
                    control_speed = 0.5
                    control_turn = 0
                    print "center : " + str(self.score_c)
            else:
                if self.score_r > self.score_c:
                    control_speed = 0
                    control_turn = -3
                    print "right : " + str(self.score_r)
                else:
                    control_speed = 0.5
                    control_turn = 0
                    print "center : " + str(self.score_c)

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('sample_bot')
    bot = SampleBot('Sample')
    rospy.Subscriber('/camera/rgb/image_raw', Image, bot.callback)
    bot.strategy()
