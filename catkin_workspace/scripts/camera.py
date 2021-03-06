#Singapore Polytechnic
#For the Singapore Autonomous Underwater Challenge 2014
#========================================================================

#Importing of Libraries==================================================
#import RPi.GPIO as GPIO
#import roslib; roslib.load_manifest('camera')
#import rospy
#from std_msgs.msg import String
#from geometry_msgs.msg import Pose2D

#import rospy
#from std_msgs.msg import String

import cv2
import numpy as np
import time

#Initialisation of Variables=============================================
state = 0
ballrelease = 0
count = 0

#Initialisation==========================================================
cambottom = cv2.VideoCapture(0)
cambottom.set(3,320)
cambottom.set(4,240)
camfront = cv2.VideoCapture(1)
camfront.set(3,320)
camfront.set(4,240)
time.sleep(2)
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(11, GPIO.OUT)

#Functions

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
    
#Main====================================================================
while(1):
    if (state == 0):
        count += 1
        _,frame = cambottom.read()
        frame = cv2.blur(frame,(3,3))
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        threshblack = cv2.inRange(hsv,np.array((0,0,0)),np.array((102,194,30)))
        threshblackdisplay = threshblack.copy()
        threshred = cv2.inRange(hsv,np.array((0,70,100)),np.array((5,255,255)))
        threshreddisplay = threshred.copy()
        contoursblack,hierarchyblack = cv2.findContours(threshblack,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contoursred, hierarchyred = cv2.findContours(threshred,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        max_area_black = 0
        max_area_red = 0
        for cnt in contoursblack:
            area_black = cv2.contourArea(cnt)
            if area_black > max_area_black:
                max_area_black = area_black
                best_cnt_black = cnt
                if max_area_black > 30:
                    M = cv2.moments(best_cnt_black)
                    bx,by = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        for cnt in contoursred:
            area_red = cv2.contourArea(cnt)
            if area_red > max_area_red:
                max_area_red = area_red
                best_cnt_red = cnt
                if max_area_red > 30:
                    N = cv2.moments(best_cnt_red)
                    rx,ry = int(N['m10']/N['m00']), int(N['m01']/N['m00'])
        if max_area_black > 10:
            cv2.circle(frame,(bx,by),5,255,-1)
            tblackx = bx - 160
            tblacky = -(by - 120)
            print 'Black tracker coordinates: X=' + str(tblackx) + ', Y=' + str(tblacky) + '.'
        if max_area_red > 30:
            cv2.circle(frame,(rx,ry),5,10,-1)
            tredx = rx - 160
            tredy = -(ry - 120)
            print 'Red tracker coordinates: X=' + str(tredx) + ', Y=' + str(tredy) + '.'
            if (max_area_red > 70000):
                ballrelease = 1
                state = 1
        cv2.imshow('Tracker',frame)
        cv2.imshow('Black Threshold Frame',threshblackdisplay)
        cv2.imshow('Red Threshold Frame',threshreddisplay)
        if (count % 15 == 0):
            cv2.imwrite('tracker' + str(count) + '.jpg',frame)
            cv2.imwrite('black' + str(count) + '.jpg',threshblackdisplay)
            cv2.imwrite('red' + str(count) + '.jpg',threshreddisplay)
            
    elif (state == 1):
        count += 1
        _,frame = camfront.read()
        frame = cv2.blur(frame,(3,3))
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        threshyellow = cv2.inRange(hsv,np.array((17,90,90)),np.array((40,255,255)))
        threshyellowdisplay = threshyellow.copy()
        contours,hierarchy = cv2.findContours(threshyellow,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        max_area_yellow = 0
        for cnt in contours:
            area_yellow = cv2.contourArea(cnt)
            if area_yellow > max_area_yellow:
                max_area_yellow = area_yellow
                best_cnt_yellow = cnt
                if max_area_yellow > 2:
                    Y = cv2.moments(best_cnt_yellow)
                    cx,cy = int(Y['m10']/Y['m00']), int(Y['m01']/Y['m00'])
                    cv2.circle(frame,(cx,cy),5,15,2)
                    tyellowx = cx - 160
                    tyellowy = -(cy - 120)
                    print 'Yellow tracker coordinates: X=' + str(tyellowx) + ', Y=' + str(tyellowy) + '.'
         
        cv2.imshow('Tracker',frame)
        cv2.imshow('Yellow Threshold Frame',threshyellowdisplay)
        if (count % 15 == 0):
            cv2.imwrite('tracker' + str(count) + '.jpg',frame)
            cv2.imwrite('yellow' + str(count) + '.jpg',threshyellowdisplay)
         
    key = cv2.waitKey(10)
    if key == 27:
        break
