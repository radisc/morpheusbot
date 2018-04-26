#!/usr/bin/env python
# -*- coding: utf-8 -*-

##
# Name: MorpheusBot
# Package: morpheusbot
# Author: Matteo Lisotto
# Date: 2015-02-18
#
# History:
# Version      Programmer       Date          Changes
# 0.0.1        Matteo Lisotto   2015-02-18    Created MorpheusBot class
# 0.0.2        Matteo Lisotto   2015-03-05    Added the services
# 0.0.3        Matteo Lisotto   2015-04-01    Added position and velocities methods
# 0.0.4        Nicola Riste'    2015-11-22    Added rotation to the marker
# 0.1          Nicola Riste'    2015-12-18    Add goTo method, transform publisher 

import roslib
from threading import Thread
from threading import Lock
from visualization_msgs.msg import Marker
from math import *
import time
import rospy
from morpheusbot.srv import *
import tf
import numpy as np
from controller import *

import random
from math import atan, sqrt

noise = True
pi = 3.1415926535
cruise_speed = 4


def rotationAngle(posx, posy, goalx, goaly):

    angle = atan((goaly - posy)/(goalx - posx))
    
    #Second quadrant
    if((goalx - posx) < 0.0 and (goaly - posy) > 0.0):
        angle = (pi + angle)#%(2*pi)
    
    #Thrid quadrant 
    if((goalx - posx) < 0.0 and (goaly - posy) < 0.0):
        angle = (pi + angle)#%(2*pi)
    
    #Fourth quadrant
    if((goalx - posx) > 0.0 and (goaly - posy) < 0.0):
            angle = (2*pi + angle)#%(2*pi)
#    else:
#        if((goaly - posy) < 0.0 and (goalx - posx) < 0.0):
#           angle = (angle + pi)%(2*pi)
           
    return angle

##
# @author Matteo Lisotto <matteo.lisotto@gmail.com>
# @version 0.0.3
# @since 2015-02-18
#
class MorpheusBot:
##
# MorpheusBot constructor
#
# @param semiaxis The lenght of the semiaxis
#
    def __init__(self, semiaxis):
        self._x = 0
        self._y = 0
        self._theta = 0
        self._vx = 0
        self._vy = 0
        self._vtheta = 0
        self._v1 = 0
        self._semiaxis = semiaxis
        self._lock = Lock()

        rospy.init_node('MorpheusBot')
        self._publisher = rospy.Publisher('MorpheusBot', Marker, queue_size=100)
        
        #Transform broadcaster
        self._br = tf.TransformBroadcaster()
        
        self.initMarker()
        thread = Thread(target=self.MoveMorpheusDaemon)
        thread.daemon = True
        thread.start()
        rospy.Service('setVelocity', SetVelocity, self.setVelocity)
        rospy.Service('setPosition', SetPosition, self.goTo)
        rospy.Service('Position', Position, self.position)
        rospy.Service('Velocities', Velocities, self.velocities) 
        
##
# Initalize the marker
#
# Initalize the marker for rviz communcation
#
    def initMarker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.6
        self.marker.scale.y = 0.3
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.5
        self.marker.color.b = 0.0
        self.marker.pose.position.x = self._x
        self.marker.pose.position.y = self._y
        self.marker.pose.position.z = 0.1
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 1
        self.marker.pose.orientation.w = 0

##
# Change the motor velocity
#
#
# @param velocityRight The speed of the right motor
# @param velocityLeft The speed of the left motor
#
    def setRPY (self, req):
        return
               

##
# Change the motor velocity
#
#
# @param velocityRight The speed of the right motor
# @param velocityLeft The speed of the left motor
#
    def setVelocity (self, req):
        with self._lock:
            self._v1 = (req.velocityRight + req.velocityLeft)/2
            self._vtheta = (req.velocityRight - req.velocityLeft)/(2*self._semiaxis)
            self._vx = cos(self._theta) * self._v1
            self._vy = sin(self._theta) * self._v1
	    return SetVelocityResponse(True)
##
# Returns latitude and longitude
#
    def position(self):
        positionLatitude = 'N' if self._y >= 0 else 'S'
        positionLongitude = 'E' if self._x >= 0 else 'W'
        return PositionResponse(self._x, positionLongitude, self._y, positionLatitude)
    
##
# Returns the vx, vy and vtheta
#
    def velocities(self):
        return VelocitiesResponse(self._vx, self._vy, self._vtheta)

##
# Moves from current position to the coordinates 
#
# @param latitude     X position of nav. point
# @param longitude    Y position of nav. point
#
    def goTo(self, req):
        bearing = rotationAngle(self._x, self._y, req.latitude, req.longitude)  #atan((req.longitude - self._y)/(req.latitude - self._x)) + (pi/2)*(1-np.sign((req.latitude - self._x))*np.sign((req.longitude - self._y)))
        distance = sqrt( (req.longitude - self._y)**2 + (req.latitude - self._x)**2 )
        D = distance
        deltaT  = distance / cruise_speed
        dstar = 10e-2
        
        
        while(distance > dstar):
            bearing = rotationAngle(self._x, self._y, req.latitude, req.longitude)
            distance= sqrt( (req.longitude - self._y)**2 + (req.latitude - self._x)**2 )
            closeness = D - distance 
            #setVel(vel, vel)
            deltaTheta = 2*((bearing%(2*pi) - self._theta%(2*pi)))
            vel = -distance**2 + (D + 0.01)*distance
            if vel>10:
                vel = 10	#1 * distance - 1*deltaTheta 
            Kt = 1
            print self._theta, bearing
            print 'deltaTheta: ', deltaTheta, 'vel: ', vel
            if(deltaTheta < pi):
                setVel(vel + deltaTheta, vel - deltaTheta)
            else:
                setVel(vel - deltaTheta, vel + deltaTheta)
            time.sleep(0.001)
        
        setVel(0, 0)
        print "Done moving"
        print self.position()
        return SetPositionResponse(True)
            
        
        '''
        print 'theta, bearing, distance, deltaT', self._theta, bearing, distance, deltaT
        print 'position: ', self._x, self._y
        print 'goTo', req.latitude, req.longitude
        
        if(bearing > self._theta%(2*pi)):
            setVel(1, -1)
            while ( bearing > self._theta%(2*pi) ):
                print abs(self._theta%(2*pi)  - bearing)
                time.sleep(0.001)
        else:
            setVel(-1, 1)
            while (bearing < self._theta%(2*pi)):
                print abs(self._theta%(2*pi) - bearing)
                time.sleep(0.001)
        
        setVel(0,0)
        setVel(4,4)     #Full Speed Ahead!
        #t = deltaT
        while(deltaT > 0.001):
            time.sleep(0.001)
            deltaT -= 0.001    #Let time pass
        setVel(0,0)
        print "Done moving"
        print self.position()
        return SetPositionResponse(True)
        '''
    
##
# Moves from current position to the coordinates 
#
# @param latitude     X position of nav. point
# @param longitude    Y position of nav. point
#
    def goTo2(self, req):
        bearing = rotationAngle(self._x, self._y, req.latitude, req.longitude)  #atan((req.longitude - self._y)/(req.latitude - self._x)) + (pi/2)*(1-np.sign((req.latitude - self._x))*np.sign((req.longitude - self._y)))
        distance= sqrt( (req.longitude - self._y)**2 + (req.latitude - self._x)**2 )
        deltaT  = distance / cruise_speed
        print 'theta, bearing, distance, deltaT', self._theta, bearing, distance, deltaT
        print 'position: ', self._x, self._y
        print 'goTo', req.latitude, req.longitude
        
        if(bearing > self._theta%(2*pi)):
            setVel(1, -1)
            while ( bearing > self._theta%(2*pi) ):
                print abs(self._theta%(2*pi)  - bearing)
                time.sleep(0.001)
        else:
            setVel(-1, 1)
            while (bearing < self._theta%(2*pi)):
                print abs(self._theta%(2*pi) - bearing)
                time.sleep(0.001)
        
        setVel(0,0)
        setVel(4,4)     #Full Speed Ahead!
        #t = deltaT
        while(deltaT > 0.001):
            time.sleep(0.001)
            deltaT -= 0.001    #Let time pass
        setVel(0,0)
        print "Done moving"
        print self.position()
        return SetPositionResponse(True)

##
# Daemon that change x,y and theta values
#
# Function that is called by a Daemon to change the values of x, y and theta
# every 1 hundredth of a second
#
    def MoveMorpheusDaemon(self):
        while not rospy.is_shutdown():
            self._x += (self._vx * 0.01)
            self._y += (self._vy * 0.01)
            self._theta += ( (self._vtheta + (random.gauss(0.0, 3.0) if noise and not (self._vx == 0 and self._vy == 0 and self._vtheta == 0) else 0.0) ) * 0.01)
            with self._lock:
                self._vx = cos(self._theta) * self._v1
                self._vy = sin(self._theta) * self._v1
            self.marker.pose.position.x = self._x
            self.marker.pose.position.y = self._y
            
            #Convert from euler to quaternion (Roll, Pitch, Yaw)    #Theta is our value of Yaw
            quat = tf.transformations.quaternion_from_euler(0,0, self._theta)
            
            ##print 'theta:', self._theta
            
            self.marker.pose.orientation.x = quat[0]
            self.marker.pose.orientation.y = quat[1]
            self.marker.pose.orientation.z = quat[2]
            self.marker.pose.orientation.w = quat[3]
            
            #self.marker.pose.orientation.w = self._theta
            self._publisher.publish(self.marker)
            T = (self._x, self._y, 0)
            self._br.sendTransform(T, quat, rospy.Time.now(), "/morpheus_bot", "/world")
            time.sleep(0.01)

    def reset ():
        self._x = self._y = self._theta =  0
        self._vx = self._vy = self._vtheta = 0
        self._v1 = 0
