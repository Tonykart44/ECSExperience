# -*- coding: utf-8 -*-
"""
Created on Wed Apr 06 22:16:49 2016

@author: Robin Amsters

This module aimes to combine functionalities from the RoombaSCI and pyrobotECS 
modules. The goal is to provide one module that can be imported to have the 
of both libraries, since RoombaSCI provides much better syntax, but pyrobotECS
provides raw encoder values.

This module is still in its very early stages and will still have many bugs, 
though many base functionalities have been tested succesfully.
"""

from RoombaSCI import RoombaAPI
from pyrobotECS import Roomba as ECSRoomba
import roomba as misc
import sys
import time
import math

###############################################################################
################################ SENSOR CLASS #################################
###############################################################################

class Sensors(object):
    def __init__(self, Roomba, pauze):
        self.API = Roomba.API
        self.ECS = Roomba.ECS
        self.rightEncoder = self.ECS.sensors.getRightEncoderCounts()
        self.leftEncoder = self.ECS.sensors.getLeftEncoderCounts()
        self.pauze = pauze
#==============================================================================
    def getBumps(self):
        """
        Function that returns the current state of the left and right bump sensors,
        True if the sensor is pressed, False if the sensor is not pressed.
        
        INPUTS:
                self = Sensor object
        OUTPUTS:
                bumpLeft = boolean that represents the state of the left bump sensor
                bumpRight = boolean that represents the state of the right bump sensor
        """    
        
        bumpLeft = self.API.sensors.bumps.left
        bumpRight = self.API.sensors.bumps.right
        
        return bumpLeft, bumpRight  
#==============================================================================
    def getDrops(self):
        """
        Function that returns the current state of the left and right wheeldrop
        sensors, True if the wheel is dropped, False if the wheel is nor dropped
        
        INPUTS:
                self = Sensor object
        OUTPUTS:
                dropLeft = boolean that represents the state of the left drop sensor
                dropRight = boolean that represents the state of the right drop sensor
        """    
        
        dropLeft = self.API.sensors.wheel_drops.left
        dropRight = self.API.sensors.wheel_drops.right
        
        return dropLeft, dropRight
#==============================================================================        
    def getCliffs(self):
        """
        Function that returns the current state of the cliff
        sensors, True if something is detected, False if nothind is detected
        
        INPUTS:
                self = Sensor object
        OUTPUTS:
                cliffLeft = state of the left most cliff sensor
                cliffFrontLeft = state of the front left cliff sensor
                cliffRight = state of the right most cliff sensor
                cliffFrontRight = state of the front right cliff sensor
                
        """
        cliffLeft = self.API.sensors.cliff.left
        cliffFrontLeft = self.API.sensors.cliff.front_left
        cliffRight = self.API.sensors.cliff.right
        cliffFrontRight = self.API.sensors.cliff.front_right
        
        return cliffLeft, cliffFrontLeft, cliffRight, cliffFrontRight
        
        
#==============================================================================
    def getOdometry(self, direction):
        """
        Function that returns the distance driven and the angle turned since the
        last call off getOdometry().
        
        INPUTS:
                self = Sensor object
                direction = direction in which roomba is driving, valid directions:
                    - 'FWD' = driving forward in a straight line
                    - 'BWD' = driving backwards in a straight line
                    - 'CCW' = turning an angle counter clockwise
                    - 'CW' = turning an angle clockwise
                
        OUTPUTS:
                ds = distance driven since last call, positive if driviving 
                     forward, negative if driving backwards.
                dth = angle turned since last call, positive if Roomba turned
                      CCW, negative if Roomba turned CW.
                
        """        
        # Roomba specific variables
        wheelCirc = 0.232 #m
        b = 0.233 #m
        ticsPerRev = 500
        ticDist = wheelCirc/ticsPerRev
        maxtics = 65535
        
        # Previous encoder values
        prevRight = self.rightEncoder
        prevLeft = self.leftEncoder

        #Current encoder values
        currentRight = self.ECS.sensors.getRightEncoderCounts()
        currentLeft = self.ECS.sensors.getLeftEncoderCounts()
        
        #Saving current encoder values
        self.rightEncoder = currentRight
        self.leftEncoder = currentLeft
        
        # Difference between current and previous values        
        if direction  == 'FWD': #Driving forward
        
            if currentRight >= prevRight:
                dRight = currentRight - prevRight
            else:
                dRight = currentRight + maxtics - prevRight #Special case for overflow
                
            if currentLeft >= prevLeft:
                dLeft = currentLeft - prevLeft
            else:
                dLeft = currentLeft + maxtics - prevLeft #Special case for overflow

        elif direction == 'BWD': #Driving backwards
        
            if currentRight <= prevRight:
                dRight = currentRight - prevRight
            else:
                dRight = -(prevRight + maxtics - currentRight) #Special case for overflow
                
            if currentLeft <= prevLeft:
                dLeft = currentLeft - prevLeft
            else:
                dLeft = -(prevLeft + maxtics - currentLeft) #Special case for overflow
                
        elif direction == 'CW': #Turning CCW
        
            if currentRight >= prevRight:
                dRight = currentRight - prevRight
            else:
                dRight = currentRight + maxtics - prevRight #Special case for overflow
                
            if currentLeft <= prevLeft:
                dLeft = currentLeft - prevLeft
            else:
                dLeft = -(prevLeft + maxtics - currentLeft) #Special case for overflow
        
        elif direction == 'CCW': #Turning CCW
        
            if currentRight <= prevRight:
                dRight = currentRight - prevRight
            else:
                dRight = -(prevRight + maxtics - currentRight) #Special case for overflow
            
            if currentLeft >= prevLeft:
                dLeft = currentLeft - prevLeft
            else:
                dLeft = currentLeft + maxtics - prevLeft #Special case for overflow
                
        else:
            
            print "Invalid input for direction, can only be 'FWD', 'BWD', 'CCW' or 'CW'."
            # If input is invalid, assing max values so driving might stop
            ds = sys.maxint
            dth = sys.maxint 
            return ds, dth          
        
        # Distance driven and angle turned since last call
        ds = ticDist*(dRight + dLeft)/2
        dth = ticDist*(dRight - dLeft)/b
        
        return ds, dth
#==============================================================================    
    def getWallSensor(self):
        """
        Function that returns the current state of the wall sensor
        
        INPUTS:
                self = Sensor object
        OUTPUTS:
                wall = current state of the wall sensor, True if a wall is detected,
                       False if no wall is detected
        """ 
        wall = self.API.sensors.wall
        
        return wall
    
#=============================================================================
    def checkSafe(self):
        """
        Function that checks whether it is safe for the roomba to drive. 
        A 'safe' situation is defined as follows:
            - No bump sensors are pressed.
            - The wheels are not extended (dropsensors are false)
        
        INPUTS:
                self = Sensor object
        OUTPUTS:
                safe = bool that says whether is is safe to drive
        
        """
        safe = False
        
        bumps = self.getBumps()
        drops = self.getDrops()
        
        if True in bumps or True in drops:
            safe = False
            print "Roomba is not in a safe position to drive."
            print "Please make sure that no bump sensors are pressed and that both wheels are on the ground."
                
        else:
            safe = True
    
        return safe       
#==============================================================================
    
###############################################################################
################################ ROOMBA CLASS #################################
###############################################################################    
class Roomba(object):
    def __init__(self, comPort, baudrate):
        self.API = RoombaAPI(comPort, baudrate)
#        print "Connecting"
#        self.API.connect()
        self.API.full()
        self.ECS = ECSRoomba(comPort)
        self.sensors = Sensors(self, 10)

        
################################### DRIVING ###################################
#==============================================================================
#    def drive(self, velocity, radius):  
#    UNDER CONSTRUCTION     
#        
#        velocity = abs(velocity*1000) #mm/s to m/s
#        velocity *= 1000
#        radius *= 1000
#    # TODO check speed limits
#    # TODO make conversions clearer
#    # TODO check backwards arc
#        if self.sensors.checkSafe():
#            
#            self.API.drive(int(velocity), int(radius))

            
#==============================================================================
    def driveStraight(self, velocity, distance):
        """
        Function that makes the Roomba drive in a straight line, at a specified
        speed for a specified distance.
        
        INPUTS:
            self = Roomba object
            velocity = velocity in m/s at which the Roomba will drive
            distance = distance in m which the Roomba will drive, negative 
                       distance means the roomba will drive backwards.
        """
        s = 0 #distance driven
        th = 0 #angle turned
        velocity = abs(velocity*1000) #mm/s to m/s
        
        if self.sensors.checkSafe():
    # TODO check speed limits
            self.API.speed = int(velocity)
            
            if distance <= 0:
                self.API.backward()
                direction = 'BWD'
                # Call getodometry once without assigning output because 
                # when quickly changing directions tics count in opposite 
                # direction on first call.
                self.sensors.getOdometry(direction)
            else:
                self.API.forward()
                direction = 'FWD'
                # Call getodometry once without assigning output because 
                # when quickly changing directions tics count in opposite 
                # direction on first call.
                self.sensors.getOdometry(direction)
    
            while abs(s) < abs(distance) and self.sensors.checkSafe():   
                     
                ds, dth = self.sensors.getOdometry(direction)
                
                s += ds
                th += dth
            
            self.stop()
        
        else:
            self.stop()
#==============================================================================
    def turnAngle(self, velocity, angle, overrideSafe = False):
        # TODO check functionality
        """
        Function that lets the Roomba turn a specified angle around it axis.
        
        INPUTS:
            velocity = the velocity in m/s at which the wheels will turn.
            angle = the angle in radians that the Roomba will turn, a positive
                    angle means that the Roomba will turn CCW, a negative angle
                    means that the Roomba will turn CW.
        
        OUTPUTS:
            none
        
        """
        s = 0 #distance driven
        th = 0 #angle turned
        velocity = abs(velocity*1000) #mm/s to m/s
        
        if self.sensors.checkSafe() or overrideSafe:
    # TODO check speed limits
            self.API.speed = int(velocity)
            
            if angle <= 0:
                self.API.spin_right()
                direction = 'CW'
                # Call getodometry once without assigning output because 
                # when quickly changing directions tics count in opposite 
                # direction on first call.
                self.sensors.getOdometry(direction)
            else:
                self.API.spin_left()
                direction = 'CCW'
                # Call getodometry once without assigning output because 
                # when quickly changing directions tics count in opposite 
                # direction on first call.
                self.sensors.getOdometry(direction)
    
            while abs(th) < abs(angle) and (self.sensors.checkSafe() or overrideSafe):                        
                ds, dth = self.sensors.getOdometry(direction)  
                
                s += ds
                th += dth
            
            self.stop()
        
        else:
            self.stop()  
#==============================================================================
    def driveArc(self, velocity, radius, angle):
        """
        Function that lets Roomba drive an arc segment, specified by a radius 
        and an angle, at a certain velocity.
        
        INPUTS:
            self = Roomba object
            velocity = speed at which roomba will drive in m/s, positive 
                       speed makes Roomba drive forward, negative speed makes
                       roomba drive backwards.
            radius  = radius of the arc segment in m. Negative radius makes 
                      radius makes Roomba turn CW, positive radius makes Roomba
                      turn CCW.
            angle  = angle of the arc segment in radians
            
        OUTPUTS:
            none
        """
        s = 0
        th = 0
        # Conversion from m to mm
        velocity *= 1000
        radius *= 1000
    # TODO check speed limits
    # TODO make conversions clearer
    # TODO check backwards arc
        if self.sensors.checkSafe():
            
            self.API.drive(int(velocity), int(radius))

            if velocity < 0:
                direction = 'BWD'
                # Call getodometry once without assigning output because 
                # when quickly changing directions tics count in opposite 
                # direction on first call.
                self.sensors.getOdometry(direction)
            else:
                direction = 'FWD'
                # Call getodometry once without assigning output because 
                # when quickly changing directions tics count in opposite 
                # direction on first call.
                self.sensors.getOdometry(direction)
            
            while abs(th) < abs(angle) and self.sensors.checkSafe():
                ds, dth = self.sensors.getOdometry(direction)
                
                s += ds
                th += dth
                
            self.stop()
        else:
            self.stop()
        
#==============================================================================
    def stop(self):
        """
        Function to stop Roomba from driving
        
        INPUTS:
            self = Roomba object
        
        OUTPUTS:
            none
        """
        try:
            self.API.stop()
        except:
            self.ECS.Stop()     
            
################################### BUTTONS ###################################
    def getClean(self):
        """
        Function that returns the current status of the clean button. 
        
        INPUTS:
            self = Roomba object
        
        OUTPUTS:
            clean = status of clean button. False if button is not pressed, 
                    True if buttton is pressed.
        
        """
        clean = self.API.sensors.buttons.clean
        return clean
#==============================================================================
    def getDock(self):
        """
        Function that returns the current status of the dock button. 
        
        INPUTS:
            self = Roomba object
        
        OUTPUTS:
            dock = status of dock button. False if button is not pressed, 
                    True if buttton is pressed.
        
        """
        dock = self.API.sensors.buttons.dock
        return dock
#==============================================================================
    def getSpot(self):
        """
        Function that returns the current status of the spot button. 
        
        INPUTS:
            self = Roomba object
        
        OUTPUTS:
            spot = status of dock button. False if button is not pressed, 
                    True if buttton is pressed.
        
        """
        spot = self.API.sensors.buttons.spot
        return spot
            
##################################### LED #####################################      
    def turnOnLED(self, color, intensity):
        """
        Function to turn on 'clean' LED in the middle of the roomba.
        
        INPUTS:
            self = Roomba object
            
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
            
        self.API.led(0, colorNum, intensityNum)
#==============================================================================
    def flashLED(self, color, numOfFlashes, flashTime):
        """
            Function to flash 'clean' LED in the middle of the roomba.
            
            INPUTS:
                self = Roomba object
                
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
            self.turnOnLED(color, 100)
            time.sleep(flashTime)
            self.turnOnLED(color, 0)
            time.sleep(flashTime)

##################################### SOUND ###################################
#==============================================================================
    def loadSong(self, songNum, song = 'BEEP'):
        """
        Function that loads a song onto the Roomba at a specified songNum.
        This song can later be played by using the function 'play'.
        
        INPUTS:
            songNum: Number (0-4) that specifies where the song will be saved
                     This is the number that later has to be used when calling play().
            song: predefined song that can be loaded, possible songs are:
                  BEEP: plays a sound simular to the charging sound.
                  TFC: plays the first 4 notes of the final countdown.
                  
        OUTPUTS:
            none
        """
        self.API.song(songNum, song)
#==============================================================================
    def play(self, songNum):
        """
        Function plays a preloaded song.
        
        INPUTS:
            songNum: location of the song, as specified in song()
        
        OUTPUTS:
            none
        """
        self.API.play(songNum)       
        
###############################################################################
#################################### MAIN #####################################
###############################################################################

if __name__ == "__main__":
    
    flags = sys.argv[1:] #Specified flags when running file
    
    baudrate = 115200
    port = "/dev/ttyUSB0"    
    
    if "-help" in flags:
        print "Possible flags: "
        print "\t-help: get general information."
        print "\t-FSM: a wall follower FSM."
        print "\t-demo: start a short demo that illustrates different"
        print "\t       aspects of the Roomba platform."
        print "\t-control: manually control Roomba with the numpad"
        print "\t-off: turn Roomba off. THIS DOES NOT SAFELY SHUTDOWN THE PI!"
                
    elif "-FSM" in flags:
        roomba = Roomba(port, baudrate)
        speed = 0.2 #[m/s]
        distance = 1 #[m]
        smallRadius = 250 #[mm]
        bigRadius = 500 #[mm]
        angle = math.pi/4 #[rad]
        angle_threshold = math.pi/2 #[rad]
        state = "start"
        
        while True:
            
            bumpLeft, bumpRight  = roomba.sensors.getBumps()
            wall = roomba.sensors.getWallSensor()
            drops = roomba.sensors.getDrops()
            cliffs = roomba.sensors.getCliffs()
            
            if state == "start":
                roomba.API.drive(int(speed*1000), 32768)
                while not (bumpRight or bumpLeft or wall): 
                    # drive -5 cm
                
                while not wall and angle < math.pi/2:
                    #turn CCW
                    
                    if True in drops:
                        state = "stop"

                else:
                    if wall:
                        state = "followwall"

                    if bumpLeft:
                        state = "turnCW"
                        
                    elif bumpRight:
                        state = "turnCCW"

            elif state == "followwall":
                
                roomba.API.drive(int(speed*1000), smallRadius)               
                currentAngle = 0
                
                while abs(currentAngle) <= abs(angle_threshold) and not bumpRight:
                    bumpLeft, bumpRight  = roomba.sensors.getBumps()
                    wall = roomba.sensors.getWallSensor()
                    drops = roomba.sensors.getDrops()
            
                    if wall:
                        roomba.API.drive(int(speed*1000), smallRadius)
                    
                    else:
                       roomba.API.drive(int(speed*1000), -smallRadius)
                    
                    ds, dth = roomba.sensors.getOdometry("FWD")
                    currentAngle += dth
                    
                if True in drops:
                    state = "stop"

                else: 
                    if wall and not bumpRight:
                        state = "followwall" 

                    elif not bumpRight:
                        state = "findwall"

                    if bumpLeft:
                        state = "turnCW"

                    elif bumpRight:
                        state = "turnCCW"
                    
            elif state == "findwall":
                
                roomba.turnAngle(speed, angle/2)
                roomba.driveStraight(speed, distance)             
                roomba.driveArc(speed, bigRadius, angle)
                
                if True in drops:
                    state = "stop"

                else:
                    if wall:
                        state = "followwall" 

                    else:
                        state = "findwall"

                    if bumpLeft:
                        state = "turnCW"

                    elif bumpRight:
                        state = "turnCCW"
                    
            elif state == "turnCW":
                roomba.turnAngle(speed, -angle/2, True)

                if True in drops:
                    state = "stop"

                else:
                    if wall or True in cliffs:
                        state = "followwall" 

                    else:
                        state = "findwall" 
                        
            elif state == "turnCCW":
                roomba.turnAngle(speed, angle/2, True)

                if True in drops:
                    state = "stop"

                else:
                    if wall or True in cliffs:
                        state = "followwall" 

                    else:
                        state = "findwall" 
                state = "followwall" 
                
            elif state == "stop":
                roomba.stop()
                sys.exit("Code execution stopped.")
    
    elif "-demo" in flags:
        roomba = Roomba(port, baudrate)
        roomba.connect()
        speed = 0.2
        distance = 0.5
        startDemo = False
        
        print "Press the 'CLEAN' button to start the demo."
        
        while not startDemo:
            if roomba.getClean():
                startDemo = True
        
        if startDemo:        
            print ""
            print "PLAYING SOUND" 
            print ""
            roomba.loadSong(1, 'BEEP')
            roomba.play(1) 
            
            print ""
            print "BLINKING LIGHT"
            print ""        
            roomba.flashLED('g', 5, 0.1)
            
            print ""
            print "DRIVING FORWARD."
            print ""
            roomba.driveStraight(speed, distance)
           
            print ""
            print "TURNING CCW."
            print ""
            roomba.turnAngle(speed, math.pi/2)
            
            print ""
            print "TURNING CW."
            print ""
            roomba.turnAngle(speed, -math.pi)
            
            print ""
            print "TURNING CCW."
            print ""
            roomba.turnAngle(speed, math.pi/2)
            
            print ""
            print "DRIVING BACKWARDS."
            print ""
            roomba.driveStraight(speed, -distance)
            
            print ""
            print "DRIVING FORWARD ARC SEGMENT."
            print ""
            roomba.driveArc(speed, 0.5, math.pi)
            
            print ""
            print "DRIVING BAWKWARDS ARC SEGMENT."
            print ""
            roomba.driveArc(-speed, 0.5, math.pi)
            
            print ""
            print "END OF DEMO."
            print ""
            roomba.loadSong(2, 'TFC')
            roomba.play(2) 
            roomba.stop()
            
    elif "-control" in flags: 
        roomba = RoombaAPI(port, baudrate)        
        misc.control(roomba)
    
    elif "-off" in flags:
        roomba = RoombaAPI(port, baudrate)   
        roomba.off()
        
    else:
        print "Please specify a valid flag, use -help for more info on possible flags."