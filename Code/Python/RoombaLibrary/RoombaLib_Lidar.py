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
import socket
import time
import math
import numpy as np

###############################################################################
################################ LIDAR CLASS ##################################
###############################################################################
class LIDAR(object):
    def __init__(self):
        self.initPort = 7071 # port used only to initialise LIDAR communication
        self.initOpcode = '201' # opcode used to initialize LIDAR communcation
        self.localPort = 7070 # port used for all other communication with LIDAR
        
    def initLidar(self):
        """
            Function that initialises LIDAR communication, very simular to 
            initLidar.m in old RoombaCommunicationToolbox
        """
        port = int(self.initPort) # Port to which to send initialisation opcode
        targetIP = socket.gethostbyname('localhost')
        targetAddr = (targetIP,port)
        outUdpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # Socket to communicate with
        outUdpSocket.sendto(self.initOpcode,targetAddr) # send opcode
        outUdpSocket.close() # close socket when done
        
    def udp_receive(self):
        """
            Function that receives a single UDP packet, very simular to 
            py_udp_receive.m in old RoombaCommunicationToolbox.
            
            This function should only be called after initLidar
        """     
        udpInPort = int(self.localPort)
        UDPTIMEOUT = 0.2
        udpInBufferLength = 6000

        localIP = socket.gethostbyname('localhost')
        localAddr = (localIP,udpInPort)
        
        inUdpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        inUdpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        inUdpSocket.settimeout(UDPTIMEOUT)
        
        # ...Bind incoming UDP socket to address of local machine.
        inUdpSocket.bind(localAddr)
        
        try:
           udpData,addr = inUdpSocket.recvfrom(udpInBufferLength)
        except:
           udpData = []
        
        # Close incoming UDP socket.        
        inUdpSocket.close()
        
        print "udpData = " ,udpData
        return udpData

    def getScan(self):
        """
            Function that returns LIDAR measurements that were sent over UDP.
            This function is very simular to getLidarScan.m in the old 
            RoombaCommunicationToolbox
        """
        t_start = time.time()
        t_end = t_start + 1
        scan = []
        while time.time() <= t_end: # Make sure no infinite loop is possible
            
            udpdata = self.udp_receive() #Receiving udp data from LIDAR as string
            udpdata = udpdata[1:len(udpdata)-2] # Remove first 2 and last 2 characters
            bytes_str = udpdata.split(',') # Split bytes at commas
#            bytes_str=map(int, udpdata.split(","))
            print "bytes_str = ", bytes_str
            
            data = [] # Convert to floats
            for byte_str in bytes_str:
                byte_float = float(byte_str)
                data.append(byte_float)
            
            n = len(data)
            
            if n%4 == 0 and n>100: # four bytes per measurement, minimum 100 measurements
                
                for i in range(0,n,4):
                    n+=1
    
                    # Convert to uint8 but keep max at 255 (like MATLAB)
                    angle_short = np.uint8([data[i],data[i+1]])
                    dist_short = np.uint8([data[i+2],data[i+3]])
                    if data[i] > 255:
                        angle_short[0] = 255
                    if data[i+1] > 255:
                        angle_short[1] = 255
                    if data[i+2] > 255:
                        dist_short[0] = 255
                    if data[i+3] > 255:
                        dist_short[1] = 255
                        
                    # Convert to uint16
                    angle_int = angle_short.view(np.uint16)
                    dist_int = dist_short.view(np.uint16)
                    
                    
                    angle_i = float((angle_int >> 1)/64) #  Shift one place and convert to float, like MATLAB code
                    angle_i = math.radians(angle_i) # convert angle to radians
                    angle_i = angle_i - math.floor(angle_i/2*math.pi)*2*math.pi # Limit angle to 2 pi
                    dist_i = float((dist_int)/4) # Convert to float, like MATLAB
                    dist_i = dist_i/1000 # convert to m
                    
                    scan.append([dist_i,angle_i])
                    
        return scan

###############################################################################
################################ SENSOR CLASS #################################
###############################################################################

class Sensors(object):
    def __init__(self, Roomba, pauze):
        self.API = Roomba.API # RoombaSCI object
        self.ECS = Roomba.ECS #Roomba object from pyRobotECS
        self.lidar = LIDAR() #Lidar sensor
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
#==============================================================================
    def getSensorValues(self):
         """
        Function that returns the value of the wall, drop and bump sensors
        (in that order)
        
        INPUTS:
                self = Sensor object
        OUTPUTS:
                sensorValues = tuple that contains  the value of the wall, 
                drop and bump sensors
        """
        
         bumps = self.getBumps()
         drops = self.getDrops()
         wall = self.getWallSensor()
         
         sensorValues = (wall, drops, bumps)
         
         return sensorValues
    
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
    def drive(self, velocity, radius):  
    # UNDER CONSTRUCTION     
        
        velocity *= 1000
        radius *= 1000
    # TODO check speed limits
    # TODO make conversions clearer
    # TODO check backwards arc      
        self.API.drive(int(velocity), int(radius))

            
#==============================================================================
    def driveStraight(self, velocity, distance, overrideSafe = False):
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
        
        if self.sensors.checkSafe()  or overrideSafe:
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
    
            while abs(s) < abs(distance) and (self.sensors.checkSafe() or overrideSafe):   
                     
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
    def driveArc(self, velocity, radius, angle, wallFollower = False):
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
        safe = self.sensors.checkSafe()
        
        if safe:
            
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
            
            while abs(th) < abs(angle) and safe:
                ds, dth = self.sensors.getOdometry(direction)
                safe = self.sensors.checkSafe()
                wall = self.sensors.getWallSensor()
                if wallFollower:
                    if velocity <= 0:
                        if not wall:
                            safe = False
                    elif velocity > 0:
                        if wall:
                            safe = False
                
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
    alternativePort = "/dev/ttyUSB1"
    
    if "-help" in flags:
        print "Possible flags: "
        print "\t-help: get general information."
        print "\t-FSM: a wall follower FSM."
        print "\t-demo: start a short demo that illustrates different"
        print "\t       aspects of the Roomba platform."
        print "\t-control: manually control Roomba with the numpad"
        print "\t-off: turn Roomba off. THIS DOES NOT SAFELY SHUTDOWN THE PI!"
    
    elif  "-FSM2" in flags:
        try:
            roomba = Roomba(port,baudrate)
        except:
            roomba = Roomba(alternativePort, baudrate)
        print "Connecting"
#        roomba.API.connect()
#        roomba.ECS.sci.full()
#        roomba.API.control()
        roomba.loadSong(1, 'BEEP')
        roomba.loadSong(2, 'TFC')
        roomba.loadSong(3, 'SW')
        roomba.play(1) 
        
        # Initial parameters
        speed = 0.2 #[m/s]
        distance = 1 #[m]
        followRadius = 350 #[mm]
        bigRadius = -2 #[m]
        angle_threshold = math.pi/2 #[rad]
        # Initial state
        state = "start"
        
        # Start program only after clean button has been pushed.
        startFSM = False
        print "Press the 'CLEAN' button to start the FSM."        
        
        while not startFSM:
            if roomba.getClean():
                startFSM = True
        
        # Main FSM loop
        while startFSM:
            print "State: ", state
            bumpLeft, bumpRight  = roomba.sensors.getBumps()
            wall = roomba.sensors.getWallSensor()
            drops = roomba.sensors.getDrops()
            
            if state == "start":
                
                # Drive in a straight line untill a wall is found
                print "Drive forward untill wall is found with bump sensor."
                roomba.driveStraight(speed, 10)               
                
                # Look around till wall if found
                roomba.API.spin_left()
                th = 0
                while not wall and not True in drops and th <= math.radians(110):
                    print "Turn CCW until wall is found with wall sensor."
                    wall = roomba.sensors.getWallSensor()   
                    drops = roomba.sensors.getDrops()
                    ds,dth = roomba.sensors.getOdometry('CCW')
                    th += dth
                    # Safety check
                    if True in drops:
                        state = "stop"
                    if wall:
                        state = "followwall"
                
            elif state == "followwall":
                th = 0
                dth = 0
                bumpLeft, bumpRight  = roomba.sensors.getBumps()
                wall = roomba.sensors.getWallSensor()
                drops = roomba.sensors.getDrops()
                tmp1,tmp2 = roomba.sensors.getOdometry("FWD") # Call odometry and dont assign values to clear buffer
                while not bumpRight and not True in drops and abs(th) <= abs(angle_threshold):
                    bumpLeft, bumpRight = roomba.sensors.getBumps()
                    wall = roomba.sensors.getWallSensor()
                    drops = roomba.sensors.getDrops()
                    print "th: ", th
            
                    if wall:
                       roomba.API.drive(int(speed*1000), followRadius)
                    
                    else:
                       roomba.API.drive(int(speed*1000), -followRadius)
                    
                    ds, dth = roomba.sensors.getOdometry("FWD")
                    print "dth: ", dth
                    th += dth
                    
                if True in drops:
                    state = "stop"

                else: 
                    if wall and not bumpRight:
                        state = "followwall" 

                    elif not bumpRight and abs(th) >= abs(angle_threshold):
                        state = "findwall"
                    print "Bumps: ", bumpLeft, bumpRight
                    print "th: ",th
                    print "Threshold: ", angle_threshold
                    if bumpRight or bumpLeft:
                        state = "turnCCW"
                        
            elif state == "turnCCW":  
                roomba.API.spin_left()
                th = 0
                wall = roomba.sensors.getWallSensor()
                bumpLeft, bumpRight = roomba.sensors.getBumps()
                drops = roomba.sensors.getDrops()
                
                while not wall and not True in drops and abs(th) <= math.radians(30):
                    wall = roomba.sensors.getWallSensor()   
                    drops = roomba.sensors.getDrops()
                    ds,dth = roomba.sensors.getOdometry('CCW')
                    th += dth
                    
                    # Safety check
                    if True in drops:
                        state = "stop"
                    elif wall:
                        state = "followwall"
                if True in drops:
                        state = "stop"
                else:
                    state = "findwall"
                    
            elif state == "findwall":                 
                roomba.turnAngle(speed, 2*angle_threshold)
                roomba.driveArc(speed, bigRadius, math.pi, True)
                
                 # Look around till wall if found
                roomba.API.spin_left()
                th = 0
                while not wall and not True in drops and th <= math.radians(110):
                    wall = roomba.sensors.getWallSensor()   
                    drops = roomba.sensors.getDrops()
                    ds,dth = roomba.sensors.getOdometry('CCW')
                    th += dth
                    # Safety check
                    if True in drops:
                        state = "stop"
                    if wall:
                        state = "followwall"
                        
                if not wall:
                    state = "stop"
                
            elif state == "stop":
                # Play song
                roomba.play(2)
                roomba.stop()
                
                # Pauze untill user input is received
                pauze = True
                while pauze:
                    if roomba.getClean():
                        print "Restarting FSM"
                        roomba.play(1)
                        state = "start"
                        pauze = False
                    if roomba.getDock() or roomba.getSpot():
                        startFSM = False
                        roomba.play(3)
                        sys.exit("Code execution stopped.")

    elif "-FSM" in flags:
        roomba = Roomba(port, baudrate)
        speed = 0.2 #[m/s]
        distance = 1 #[m]
        followRadius = 2000 #[mm]
        bigRadius = 2 #[mm]
        angle_threshold = math.pi/2 #[rad]
        state = "start"
        
        startFSM = False
        
        print "Press the 'CLEAN' button to start the FSM."        
        
        while not startFSM:
            if roomba.getClean():
                startFSM = True
        
        while startFSM:   
            print "State: ", state                 
            bumpLeft, bumpRight  = roomba.sensors.getBumps()
            wall = roomba.sensors.getWallSensor()
            drops = roomba.sensors.getDrops()
            s = 0
            th = 0
            
            if state == "start": #find outer wall
            # Drive forward until a wall is found with a bump sensor (or wall sensor),
            # then turn CCW until a wall is found with the wall sensor, turn another 
            # 30 degrees CCW, then drive forward for [1m]. 
            # If a new wall is found with the right or front bump sensor within 
            # this distance, the Roomba has found the outer wall 
            # and will start 'followwall', otherwise the inner wall is found 
            # and the Roomba will start 'start_innerwall'.
      
                print "Drive forward untill wall is found with bump (or wall)"
                roomba.driveStraight(speed, 10)

                if (bumpRight or bumpLeft or wall): 
                    # drive forward until a wall is found
                    
                    roomba.stop()
                    
                roomba.API.spin_left()
                th = 0
                while not wall and not True in drops and th <= math.radians(110):
                    print "Turn CCW until wall is found with wall sensor."
                    # turn angle until a wall is found with wall sensor
                    
                    wall = roomba.sensors.getWallSensor()   
                    drops = roomba.sensors.getDrops()
                    ds,dth = roomba.sensors.getOdometry('CCW')
                    th += dth
                    if True in drops:
                        state = "stop"
                    elif wall:
                        print "Driving backwards."
                        roomba.driveStraight(speed, -0.05, True)
                        state = "followwall"
                # turn 30 degrees
                print "Turn 30 deg"
                roomba.turnAngle(speed,math.pi/6)
                
                
                
#                roomba.API.drive(int(speed*1000), 500)
#                s = 0
#                
#                while not wall and not bumpRight and not True in drops and s < 1:
#                    print "Drive forward for 1 m or until wall is found first"
#                    # drive forward for 1 m or until wall is found first
#                    
#                    bumpLeft, bumpRight  = roomba.sensors.getBumps()
#                    wall = roomba.sensors.getWallSensor()
#                    ds,dth = roomba.sensors.getOdometry('FWD')
#                    s += ds
#                    
#                    drops = roomba.sensors.getDrops()
#                    if True in drops:
#                        state = "stop"
#                        print "State: ", state 
#
#                if bumpRight or (bumpRight and bumpLeft):
#                    # outer wall is found
#                    
#                    state = "followwall"
#                    print "State: ", state 
#                else:
#                    # inner wall is found
#                    
#                    state = "start_innerwall"
#                    print "State: ", state                                   
            
#                if True in drops:
#                        state = "stop"
#                        print "State: ", state 
#                else:
#                    if wall:
#                        state = "followwall"
#                        print "State: ", state 
#
#                    if bumpLeft:
#                        state = "turnCW"
#                        print "State: ", state 
#                        
#                    elif bumpRight:
#                        state = "turnCCW"
#                        print "State: ", state 
                    
#            elif state == "start_innerwall":
#                # Roomba is positioned at the inner wall, turns 90 deg,
#                # then drives forward until the outer wall is found (with bump sensor),
#                # then turn until wall is found with wall sensor
#                
#                print "State: ", state 
#                drops = roomba.sensors.getDrops()
#                if True in drops:
#                    state = "stop"
#                    print "State: ", state 
#                else:
#                    roomba.turnAngle(int(speed),math.pi/2)
#                    roomba.driveStraight(speed,10)
#                    roomba.API.spin_left()
#                    while not wall and not True in drops:
#                        wall = roomba.sensors.getWallSensor()
#                        drops = roomba.sensors.getDrops()
#                        
#                        if wall:
#                            state = "followwall"
#                            print "State: ", state 
#                        elif True in drops:
#                            state = "stop"
                        
            elif state == "followwall":
                print "State: ", state 
                roomba.API.drive(int(speed*1000), followRadius)               
                th = 0
                bumpLeft, bumpRight  = roomba.sensors.getBumps()
                wall = roomba.sensors.getWallSensor()
                drops = roomba.sensors.getDrops()
                
                while not bumpRight and not True in drops and abs(th) <= abs(angle_threshold):
                    bumpLeft, bumpRight  = roomba.sensors.getBumps()
                    wall = roomba.sensors.getWallSensor()
                    drops = roomba.sensors.getDrops()
            
                    if wall:
                        roomba.API.drive(int(speed*1000), followRadius)
                    
                    else:
                       roomba.API.drive(int(speed*1000), -followRadius)
                    
                    ds, dth = roomba.sensors.getOdometry("FWD")
                    th += dth
                    
                if True in drops:
                    state = "stop"

                else: 
                    if wall and not bumpRight:
                        state = "followwall" 

                    elif not bumpRight:
                        state = "findwall"

                    if bumpRight or bumpLeft:
                        state = "turnCCW"
                    
            elif state == "findwall":
                print "State: ", state                 
                roomba.turnAngle(speed, 2*angle_threshold)
                roomba.driveArc(speed, bigRadius, math.pi)
                
                if True in drops:
                    state = "stop"

                else:
                    if wall:
                        state = "followwall"

                    else:
                        state = "findwall"

                    if bumpRight or bumpLeft:
                        state = "turnCCW"

            elif state == "turnCCW":
                print "State: ", state
                roomba.API.spin_left()
                th = 0
                
                while not wall and not True in drops and th <= 2*math.pi:
                    wall = roomba.sensors.getWallSensor()
                    drops = roomba.sensors.getDrops()
                    
                    ds, dth = roomba.sensors.getOdometry("CCW")
                    th += dth
                    
                if True in drops:
                    state = "stop"

                else:
                    if wall:
                        state = "followwall" 

                    else:
                        state = "findwall"
                            
                roomba.stop()
                state = "findwall"
                
            elif state == "stop":
                roomba.stop()
                startFSM = False
                sys.exit("Code execution stopped.")

    
    elif "-demo" in flags:
        roomba = Roomba(port, baudrate)
        #roomba.connect()
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
    
    elif "-lidardemo" in flags:
        roomba = Roomba(port, baudrate)
        roomba.sensors.lidar.initLidar()
        scan = roomba.sensors.lidar.getScan()
        
        print "LIDAR measurements: "
        for measurement in scan:
            print measurement
        
        
    elif "-control" in flags: 
        roomba = RoombaAPI(port, baudrate)        
        misc.control(roomba)
    
    elif "-off" in flags:
        roomba = RoombaAPI(port, baudrate)   
        roomba.off()
        
    else:
        print "Please specify a valid flag, use -help for more info on possible flags."