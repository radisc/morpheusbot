#!/usr/bin/env python
##
# Name: controller
# Package: morpheusbot
# Author: Nicola Riste'
# Date: 2015-02-18
#
# History:
# Version      Programmer       Date          Changes
# 0.0.1        Nicola Riste'   2015-11-22     Created class for arrow keys control
# 0.0.2        Nicola Riste'   2015-12-10     add goTo method
# 0.2          Nicola Riste'   2015-12-18     Change frame_id from "morpheubot" to "world"


from visualization_msgs.msg import Marker

import cv2 as cv
import rospy
import numpy as np
import pygame
from pygame.locals import *

# this solution will work only in Windows, as msvcrt is a Windows only package
 
import thread
import time
import morpheusbot.MorpheusBot
from morpheusbot.srv import *


##
#Function to set velocities for the robot
#
# @param Vs The speed of the left motor
# @param Vd The speed of the right motor
#
def setVel(Vs, Vd):
    rospy.wait_for_service('setVelocity')
    try:
        set_velocity = rospy.ServiceProxy('setVelocity', SetVelocity)
        set_velocity(Vs, Vd)
        print 'velocity set to ', Vs, Vd 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def goTo(X, Y):
    rospy.wait_for_service('setPosition')
    try:
        print "Order issued to move to ", X, Y
        initMarker(X, Y)
        set_position = rospy.ServiceProxy('setPosition', SetPosition)
        set_position(X, Y)
    except rospy.ServiceException, e:
        print "Move service call failed"

##
# Initalize the marker
#
# Initalize the marker for rviz communcation
#
def initMarker(x, y):
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    marker.color.r = 0.5
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 1
    marker.pose.orientation.w = 0
    _publisher.publish(marker)
 
if __name__ == '__main__':
   
    rospy.init_node('MorpheusController')
   
    img = np.zeros((100,300,3), np.uint8)
    cv.namedWindow('Keep this window focused to control the robot') 
     
    _publisher = rospy.Publisher('MorpheusController', Marker, queue_size=100)

        
    while not rospy.is_shutdown():
        cv.imshow('Keep this window focused to control the robot', img) 
        k = cv.waitKey(1) & 0xFF    #0xFF is necessary on 64bit systems
        
        
        #List of waitpoints
        if k == ord('1'):
            #print 'avanti'
            goTo(9, 9)
            #setVel(2, 2)
        
        if k == ord('2'):
            #print 'avanti'
            goTo(-5, 5)
            #setVel(2, 2)    

        if k == ord('3'):
            #print 'avanti'
            goTo(-3, -7)
            #setVel(2, 2)

        if k == ord('4'):
            #print 'avanti'
            goTo(2.2, -10.7)
            #setVel(2, 2)
            
        if k == ord('5'):
            #print 'avanti'
            goTo(10, 10)
            #setVel(2, 2)
        
        if k == ord('6'):
            #print 'avanti'
            goTo(5, -5)
            #setVel(2, 2)    

        if k == ord('7'):
            #print 'avanti'
            goTo(3, -7)
            #setVel(2, 2)

        if k == ord('8'):
            #print 'avanti'
            goTo(2.2, 10.7)
            #setVel(2, 2)        
            
        if k == ord('0'):
            #print 'avanti'
            goTo(0, 0)
            #setVel(2, 2)
        
        if k == ord('w'):
            print 'avanti'
            #goTo(10, 10)
            setVel(4, 4)
        if k == ord('s'):
            print 'indietro'
            setVel(-1, -1)
        if k == ord('a'):
            setVel(1, -1)
            print 'sinistra'
        if k == ord('d'):
            setVel(-1, 1)
            print 'destra'
        if k == ord(' '):
            setVel(0, 0)
            print 'stop'
        
       
        elif k == 27:
            setVel(0, 0)
            break
        
        
    