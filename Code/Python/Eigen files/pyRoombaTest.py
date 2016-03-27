# -*- coding: utf-8 -*-
"""
Created on Sat Mar 26 15:04:17 2016

@author: robin

"""

from RoombaSCI import RoombaAPI
import time

################################ FUNCTIONS ####################################

#==============================================================================
def getBumps(roomba):
    bumpLeft = roomba.sensors.bumps.left
    bumpRight = roomba.sensors.bumps.right
    
    return bumpLeft, bumpRight
    
#==============================================================================
def getChargeLevel(roomba):
    charge = roomba.sensors.charge
    capacity = roomba.sensors.capacity
    chargeLevel = charge/capacity*100
    
    return chargeLevel
#==============================================================================
def getDrops(roomba):
    dropLeft = roomba.sensors.wheel_drops.left
    dropRight = roomba.sensors.wheel_drops.right
    
    return dropLeft, dropRight
#==============================================================================
def getOdometry(roomba):
    distance = float(roomba.sensors.distance)
    angle = float(roomba.sensors.angle)
    
    return distance, angle
    
#==============================================================================    
def getWallSensor(roomba):
    wall = roomba.sensors.wall
    
    return wall
#============================================================================== 
def printSensors(roomba):
    
        print " "
        print "Odometry: "
        distance, angle = getOdometry(roomba)
        print "\tdistance %.5f" % (distance)
        print "\tangle: %.5f" % (angle)
        
        print " "
        print "wall: ", getWallSensor(roomba)
        print " "
        
        print " "
        print "charge: %.5f '%'" % (getChargeLevel(roomba))
        print " "          
#==============================================================================

################################## DEMO #######################################
if __name__ == "__main__":
    
    # Making RoombaAPI object
    port = "/dev/ttyUSB0"
    baudrate = 115200
    roomba = RoombaAPI(port, baudrate)
    
    print "Connecting ..."
    roomba.connect()
    
    print "Setting Roomba to full."
    roomba.full()
    roomba.play(3) #Only this songNum produces sound
    print "I am alive if you just heard me beep"
           
    print "Driving forward."
    roomba.forward()
    
    time_start = time.time()
    time_end = time_start + 5
    
    while time.time() <= time_end:
        printSensors(roomba)
        time.sleep(0.05)
        
    roomba.stop()
    time.sleep(0.5)
    
    print "Turning left."
    roomba.spin_left()
    time_start = time.time()
    time_end = time_start + 5
    
    while time.time() <= time_end:
        printSensors(roomba)
        time.sleep(0.25)
        
    roomba.stop()
    print "Stopped driving."
