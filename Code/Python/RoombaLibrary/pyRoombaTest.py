# -*- coding: utf-8 -*-
"""
@author: Robin Amsters
Last updated on 27/03/2016

A python library that provides an alternative to the provided pyRobotECS 
library, these functions resemble the MATLAB syntax more closely, so that 
less work is involved when translating simulation code.

Known issues:
- getChargeLevel returns zero, this has not been fixed yet because it is not an
  important feature

- The angle returned by getOdometry only has a resolution of 1 radians (+-60°)
  this is only a problem when using turnAngle, inside this function the angle 
  turned is estimated in stead of measured. Please only use getOdometry for
  distance information, as this has a resolution of +- 1cm, which is acceptable
  in most cases.

"""

from RoombaSCI import RoombaAPI
import time
import math

###############################################################################
################################## SENSORS ####################################
###############################################################################

################################# SENSOR DATA #################################
#==============================================================================
def getBumps(roomba):
    """
    Function that returns the current state of the left and right bump sensors,
    True if the sensor is pressed, False if the sensor is not pressed.
    
    INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
    OUTPUTS:
            bumpLeft = boolean that represents the state of the left bump sensor
            bumpRight = boolean that represents the state of the right bump sensor
    """    
    
    bumpLeft = roomba.sensors.bumps.left
    bumpRight = roomba.sensors.bumps.right
    
    return bumpLeft, bumpRight  
#==============================================================================
def getChargeLevel(roomba):
    """
    Function that returns the current charge level of the roomba
    
    INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
    OUTPUTS:
            chargeLevel = level of charge left on the roomba in %
    """ 
    charge = roomba.sensors.charge
    capacity = roomba.sensors.capacity
    chargeLevel = charge/capacity*100
    
    return chargeLevel
#==============================================================================
def getDrops(roomba):
    """
    Function that returns the current state of the left and right wheeldrop
    sensors, True if the wheel is dropped, False if the wheel is nor dropped
    
    INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
    OUTPUTS:
            dropLeft = boolean that represents the state of the left drop sensor
            dropRight = boolean that represents the state of the right drop sensor
    """    
    
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
    """
    Function that returns the current state of the wall sensor
    
    INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
    OUTPUTS:
            wall = current state of the wall sensor, True if a wall is detected,
                   False if no wall is detected
    """ 
    wall = roomba.sensors.wall
    
    return wall
    
##################################### LED #####################################
#==============================================================================      
def turnOnLED(roomba, color, intensity):
    """
    Function to turn on 'clean' LED in the middle of the roomba.
    
    INPUTS:
        roomba = RoomaAPI object (see RoombaSCI library)
        
        color = string that selects the color that the LED will have, allowed 
                inputs are:
                
                    'g'  = green
                    'y' = yellow
                    'o' = orange
                    'r' = red 
        
        intensity = number from 0-100 that represents the intensity, 0 means 
                    LED is off, 100 is maximum intensity.
                    
    OUTPUTS:
       none
    """
    
    if color == 'g':
        colorNum = 0
    elif color == 'y':
        colorNum = 8
    elif color == 'o':
        colorNum = 128
    elif color == 'r':
        colorNum = 255
    else:
        print "Invalid input for color, setting color to red."
        colorNum = 255
        
    intensityNum = int(255*intensity)/100 #Converting intensity to int bewteen 0 and 255
        
    roomba.led(0, colorNum, intensityNum)
#==============================================================================
def flashLED(roomba, color, numOfFlashes, flashTime):
    """
        Function to flash 'clean' LED in the middle of the roomba.
        
        INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
            
            color = single character that selects the color that the LED will
                    have, allowed inputs are:
                    
                        'g'  = green
                        'y' = yellow
                        'o' = orange
                        'r' = red 
           numOfFlashes = number of times the LED will flash.
           
           flashTime = time in seconds between flashes
           
        OUTPUTS:
           none
    """


    
    for i in range(numOfFlashes):
        turnOnLED(roomba, color, 100)
        time.sleep(flashTime)
        turnOnLED(roomba, color, 0)
        time.sleep(flashTime)
    
################################ SENSORS MISC #################################
#============================================================================== 
def printSensors(roomba):
    """
    Function that provides some information about the current information 
    returned by the sensors. This is mostly included for demo purposes, but can
    also be used for debugging purposes.
    
    INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
    OUTPUTS:
            None, information is printed on terminal
    
    """
    
    print " "
    print "Odometry: "
    distance, angle = getOdometry(roomba)
    print "\tdistance %.5f" % (distance)
    print "\tangle: %.5f" % (angle)
    
    print " "
    print "wall: ", getWallSensor(roomba)
    print " "
    
    print " "
    print "charge: %.5f " % (getChargeLevel(roomba)) + "%"
    print " "   
#============================================================================== 
def checkSafe(roomba):
    """
    Function that checks whether it is safe for the roomba to drive. 
    A 'safe' situation is defined as follows:
        - No bump sensors are pressed.
        - The wheels are not extended (dropsensors are false)
    
    INPUTS:
            roomba = RoomaAPI object (see RoombaSCI library)
    OUTPUTS:
            safe = bool that says whether is is safe to drive
    
    """
    safe = False
    
    bumps = getBumps(roomba)
    drops = getDrops(roomba)
    
    if True in bumps or True in drops:
        safe = False
    else:
        safe = True

    return safe
        
###############################################################################
################################## ACTUATORS ##################################
###############################################################################
        
################################### DRIVING ###################################
#==============================================================================
def driveStraight(roomba, velocity, distance):
    s = 0 #distance driven
    th = 0 #angle turned
    velocity = abs(velocity*1000) #mm/s to m/s
    distance *= 100 #m to cm
    
    if checkSafe(roomba) or True:
# TODO check speed limits
        roomba.speed = int(velocity)
        

        if distance <= 0:
            roomba.backward()
        else:
            roomba.forward()

        while abs(s) < abs(distance) and checkSafe(roomba):            
            time.sleep(0.2)            
            ds, dth = getOdometry(roomba)
            
            s += ds
            th += dth
            
            printSensors(roomba)
        
        roomba.stop()
    
    else:
        roomba.stop()
#==============================================================================
def driveArc(roomba, velocity, radius, angle):
    s = 0
    th = 0
    velocity = abs(velocity*1000)
    radius = (radius*1000)
    dt = 0.2
# TODO check speed limits
# TODO make conversions clearer
    if checkSafe(roomba):
        roomba.drive(int(velocity), int(radius))
        
        print radius/1000
        
        while abs(s) < abs(radius/10*angle) and checkSafe(roomba):
            time.sleep(dt)
            ds, dth = getOdometry(roomba)
            ds_est, dth_est = estimateOdometry(velocity, dt)
            
            s += ds
            th += dth_est
            
            print s, th
            
        roomba.stop()
    else:
        roomba.stop()
    
#==============================================================================
def turnAngle(roomba, velocity, angle):
    s = 0 #distance driven
    th = 0 #angle turned
    velocity = abs(velocity*1000) #mm/s to m/s
    dt = 0.2
    
    if checkSafe(roomba):
# TODO check speed limits
        roomba.speed = int(velocity)
        
        if angle <= 0:
            roomba.spin_right()
        else:
            roomba.spin_left()

        while abs(th) < abs(angle) and checkSafe(roomba):            
            time.sleep(dt)            
            ds, dth = getOdometry(roomba)  
            ds_est, dth_est = estimateOdometry(velocity, dt)
            
            s += ds
            th += dth_est
            
            print s, th
        
        roomba.stop()
    
    else:
        roomba.stop()        

###############################################################################        
################################## MISC #######################################
###############################################################################
def estimateOdometry(speed, samplingRate):
# TODO edit to work on more than 2 special cases (straight and turn axis)
    b = 0.233
    ds_est = speed*samplingRate
    dth_est = ((4*speed/1000)/b)*samplingRate
    
    return ds_est, dth_est

###############################################################################        
################################## DEMO #######################################
###############################################################################
        
if __name__ == "__main__":
# TODO add demo as an option
# TODO add help as an option    
    # Making RoombaAPI object
    port = "/dev/ttyUSB0"
    baudrate = 115200
    roomba = RoombaAPI(port, baudrate)
    
#    print "Connecting ..."
#    roomba.connect()
    
    print "Setting Roomba to full."
    roomba.full()
    time.sleep(0.5)
    print "Testing song"
    roomba.song(1,'TFC')
    roomba.play(1) #Only this songNum produces sound
    time.sleep(2)
    
    print "Flashing light"
    flashLED(roomba, 'g', 5, 0.1)
    
    print "Driving forward."
#    driveStraight(roomba, 0.5, 0.5)
    roomba.forward()
    time.sleep(0.5)
    printSensors(roomba)
    time.sleep(1)
    printSensors(roomba)
#    
#    print "Turning left."
#    turnAngle(roomba, 0.2, math.pi/2)   
#    
#    print "Turning Right."
#    turnAngle(roomba, 0.2, -math.pi)
#    
#    print "Turning left."
#    turnAngle(roomba, 0.2, math.pi/2)   
#    
#    print "Driving Backward."
#    driveStraight(roomba, 0.5, -0.5)
    
#    print "Driving Arc."
#    driveArc(roomba, 0.3,0.25,math.pi)
#    roomba.spin_left()
#    time_start = time.time()
#    time_end = time_start + 5
#    
#    while time.time() <= time_end:
#        printSensors(roomba)
#        time.sleep(1)
#        
    roomba.stop()
    print "Stopped demo."
