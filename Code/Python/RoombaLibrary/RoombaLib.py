 # -*- coding: utf-8 -*-
"""
Created on Wed Apr 06 22:16:49 2016

@author: Robin Amsters
email: robinamsters@gmail.com

This module aimes to combine functionalities from the RoombaSCI and pyrobotECS 
modules. The goal is to provide one module that can be imported to have the best
of both libraries, since RoombaSCI provides much easier syntax, but pyrobotECS
provides raw encoder values.

This module is still in its  early stages and will still have bugs (most of which 
are focussed in the LIDAR class, as this particulary tricky to debug), 
though many base functionalities have been tested succesfully.

Some documentation (dutch) can be found at: https://ecsrobincederic.wordpress.com/

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
            initLidar.m in old RoombaCommunicationToolbox (MATLAB)
            
            INPUTS:
                None
            OUTPUTS:
                None
            
        """
        port = int(self.initPort) # Port to which to send initialisation opcode
        targetIP = socket.gethostbyname('localhost')  # Communication is done with a socket on the pi
        targetAddr = (targetIP,port) # Get target address
        outUdpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # Socket to communicate with
        outUdpSocket.sendto(self.initOpcode,targetAddr) # send opcode
        outUdpSocket.close() # close socket when done
        
    def udp_receive(self):
        """
            Function that receives a single UDP packet, very simular to 
            py_udp_receive.m in old RoombaCommunicationToolbox.
            
            This function should only be called after initLidar
            
            INPUTS:
                None
            OUTPUTS:
                udpData = data as received from udp_lidar (present on pi) over 
                          port 7070.
        """     
        udpInPort = int(self.localPort) # Port to get data from
        UDPTIMEOUT = 2 
        udpInBufferLength = 8000 

        localIP = socket.gethostbyname('localhost')
        localAddr = (localIP,udpInPort)
        
        inUdpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # Socket to communicate with
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
        
        return udpData

    def getScan(self,t_wait):
        """
            Function that returns LIDAR measurements that were sent over UDP.
            This function is very simular to getLidarScan.m in the old 
            RoombaCommunicationToolbox
            
            INPUTS:
                t_wait = maximum allowed time to wait on LIDAR measurements [s]
            
            OUTPUTS:
                scan = LIDAR data, arranged in a list of measurements. Each
                       measurement consists of a distance [m] and an angle at
                       which this distance was found [rad].
        """
        self.initLidar()
        t_start = time.time()
        t_end = t_start + t_wait
        scan = []
        while time.time() <= t_end and len(scan) < 50: # Make sure no infinite loop is possible
            udpdata = self.udp_receive() #Receiving udp data from LIDAR as string
            udpdata = udpdata[1:len(udpdata)-2] # Remove first 2 and last 2 characters
            
            data = [] # Convert to floats
            try:
                bytes_str = udpdata.split(',') # Split bytes at commas
                for byte_str in bytes_str:
                    try:
                        byte_float = float(byte_str)
                        data.append(byte_float)
                    except:
                        data = []
            except:
                data = []
            
            n = len(data)
            
            if n%4 == 0 and n>50: # four bytes per measurement, minimum 100 measurements
                
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
                    angle_i = angle_i - math.floor(angle_i/(2*math.pi))*2*math.pi # Limit angle to 2 pi
                    dist_i = float((dist_int)/4) # Convert to float, like MATLAB
                    dist_i = dist_i/1000 # convert to m
                    
                    scan.append([dist_i,angle_i])

                # Check whether received data makes sense
                # If there are some distance measurements that are 0, chances
                # are that the measurement is good 
                i = 0 
                while i < len(scan):
                    point = scan[i]
                    dist = point[0]
                    if dist <= 0:
                        del scan[-1] # remove last element from scan as it is usually wrong
                        return scan
                    else:
                        i +=1
    
                if i >= len(scan):
                    print "Nothing usefull received from LIDAR, trying again."
                    data = []
                    scan = []
        if len(scan) == 0:
            print "LIDAR scan failed, no measurements were returned."

        return scan
	
    def getShortestDistance(self, scan):
        """
            Function that returns the shortest distance of all measurements
            in a LIDAR scan.
            
            INPUTS:
                scan = LIDAR measurements as returned by getScan
            
            OUTPURS:
                r_min = minimum range presen in scan (whilst ignoring 0 
                        measurements)
        """
	r_min = 10 #initialize as big number, to be replaced in loop
	if len(scan) > 0:
		
		for point in scan:
			dist = point[0]
			if dist >= 0.1 and dist <= r_min: # Ignore zeros
				r_min = dist
	
	return r_min

###############################################################################
################################ SENSOR CLASS #################################
###############################################################################

class Sensors(object):
    def __init__(self, Roomba, pauze):
        self.API = Roomba.API # RoombaSCI object
        self.ECS = Roomba.ECS # Roomba object from pyRobotECS
        self.lidar = LIDAR() #Lidar object
        # encoders from pyRobotECS
        self.rightEncoder = self.ECS.sensors.getRightEncoderCounts() 
        self.leftEncoder = self.ECS.sensors.getLeftEncoderCounts()
        self.pauze = pauze # Might be unnecessary, unclear at this point in time
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
                    - 'FWD' = driving forward in a straight line or arc
                    - 'BWD' = driving backwards in a straight line or arc
                    - 'CCW' = turning an angle counter clockwise
                    - 'CW' = turning an angle clockwise
                
        OUTPUTS:
                ds = distance driven since last call, positive if driviving 
                     forward, negative if driving backwards.
                dth = angle turned since last call, positive if Roomba turned
                      CCW, negative if Roomba turned CW.
                
        """     
        # Roomba specific variables
        wheelCirc = 0.232 # circumference of Roomba wheels [m]
        b = 0.233 # Wheelbase of Roomba [m]
        ticsPerRev = 500
        ticDist = wheelCirc/ticsPerRev
        maxtics = 65535 # max tics before overflow [m]
        
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
#            print "Roomba is not in a safe position to drive."
#            print "Please make sure that no bump sensors are pressed and that both wheels are on the ground."
                
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
                none
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
        self.ECS.Control() # Set rto control mode
        self.ECS.sci.full() # Set to full mode
        self.sensors = Sensors(self, 10)

################################### DRIVING ###################################
#==============================================================================
    def drive(self, velocity, radius):  
    # UNDER CONSTRUCTION     
        
        velocity *= 1000
        radius *= 1000
    # TODO check speed limits
    # TODO make conversions clearer
        self.API.drive(int(velocity), int(radius))

            
#==============================================================================
    def driveStraight(self, velocity, distance, overrideSafe = False):
        """
        Function that makes the Roomba drive in a straight line, at a specified
        speed for a specified distance.
        
        INPUTS:
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
        dth = 0
        ds  = 0
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
                # Handle unexpected odometry errors
                if abs(dth) >= 100: 
                    dth = 0
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
        dth = 0
        ds  = 0
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
                
                # Handle unexpected odometry errors
                if abs(dth) >= 100: 
                    dth = 0
#                print "ds: ", ds
#                print "dth: ", dth
                safe = self.sensors.checkSafe()
                wall = self.sensors.getWallSensor()
                if wallFollower:
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
    port = "/dev/ttyUSB1"    
    alternativePort = "/dev/ttyUSB0"
    
    if "-help" in flags:
        print "Possible flags: "
        print "\t-help: get general information."
        print "\t-FSM: a wall follower FSM."
        print "\t-demo: start a short demo that illustrates different"
        print "\t       aspects of the Roomba platform."
        print "\t-lidardemo: shows lidar functionality."
        print "\t-control: manually control Roomba with the numpad"
        print "\t-off: turn Roomba off. THIS DOES NOT SAFELY SHUTDOWN THE PI!"
    
    elif  "-FSM" in flags:
        print "Connecting"
        try:
            roomba = Roomba(port,baudrate)
        except:
            roomba = Roomba(alternativePort, baudrate)

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
        followRadius = 750 #[mm]
        bigRadius = -2 #[m]
        angle_threshold = math.radians(80) #[rad]
        state = "start" # Initial state
        
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

                roomba.driveStraight(speed, 10)    # Drive forward untill wall is found with bump sensor."
                roomba.turnAngle(speed/2, -math.pi/2, True) # Turn CW untill wall is found with bump sensor."
                roomba.driveStraight(speed, 10)    # Drive forward untill wall is found with bump sensor."
                
                # Look around till wall if found
                roomba.API.spin_left()
                th = 0
                while not wall and not True in drops and th <= math.pi:
                    wall = roomba.sensors.getWallSensor()   
                    drops = roomba.sensors.getDrops()
                    ds,dth = roomba.sensors.getOdometry('CCW')
                    th += dth
                    # Safety check
                    if True in drops:
                        state = "stop"
                    if wall:
                        tmp, tmp2 = roomba.sensors.getOdometry('CCW')
                        roomba.driveStraight(speed, -0.05, True)  # Drive backwards away from wall
                        roomba.turnAngle(speed/2, math.pi/4, True) # Turn CCW untill wall is found with bump sensor."
                        state = "followwall"
                
                if not wall:
                    state = "stop" # After while stop roomba when no wall is found
                
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
                    ds,dth = roomba.sensors.getOdometry("FWD")
                    th +=dth
            
                    if wall:
                       roomba.API.drive(int(speed*1000), followRadius)
                    
                    else:
                       roomba.API.drive(int(speed*1000), -followRadius)
                    
                    ds, dth = roomba.sensors.getOdometry("FWD")
                    
                if True in drops:
                    state = "stop"

                else: 
                    if wall and not bumpRight:
                        state = "followwall" 

                    elif not bumpRight and abs(th) >= abs(angle_threshold):
                        state = "findwall"
                
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
                else:
                    state = "findwall"
                    
            elif state == "findwall":                
                tmp1,tmp2 = roomba.sensors.getOdometry('CCW')
                roomba.turnAngle(speed/2, math.radians(160))                 

                tmp1,tmp2 = roomba.sensors.getOdometry('FWD')
                roomba.driveArc(speed, bigRadius, math.pi, True)
                
                if bumpRight or bumpLeft:
                     # Look around till wall if found
                    roomba.API.spin_left()
                    th = 0
                    dth = 0
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
                else:
                     state = "findwall"  
                
                
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

    elif  "-FSMLidar" in flags:
        print "Connecting"
        try:
            roomba = Roomba(port,baudrate)
            roomba.sensors.lidar.initLidar()
        except:
            roomba = Roomba(alternativePort, baudrate)
            roomba.sensors.lidar.initLidar()
        
        roomba.loadSong(1, 'BEEP')
        roomba.loadSong(2, 'TFC')
        roomba.loadSong(3, 'SW')
        roomba.play(1) 
        
        # Initial parameters
        speed = 0.2 #[m/s]
        distance = 1 #[m]
        followRadius = 750 #[mm]
        bigRadius = -2 #[m]
        angle_threshold = math.radians(80) #[rad]
        t_wait = 2 # Time to wait for lidar measurements
        state = "start" # Initial state
        
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

                roomba.driveStraight(speed, 10)    # Drive forward untill wall is found with bump sensor."
                roomba.turnAngle(speed/2, -math.pi/2, True) # Turn CW untill wall is found with bump sensor."
                roomba.driveStraight(speed, 10)    # Drive forward untill wall is found with bump sensor."
                
                # Look around till wall if found
                roomba.API.spin_left()
                th = 0
                while not wall and not True in drops and th <= math.pi:
                    wall = roomba.sensors.getWallSensor()   
                    drops = roomba.sensors.getDrops()
                    ds,dth = roomba.sensors.getOdometry('CCW')
                    th += dth
                    # Safety check
                    if True in drops:
                        state = "stop"
                    if wall:
                        tmp, tmp2 = roomba.sensors.getOdometry('CCW')
                        roomba.driveStraight(speed, -0.05, True)  # Drive backwards away from wall
                        roomba.turnAngle(speed/2, math.pi/4, True) # Turn CCW untill wall is found with bump sensor."
                        state = "followwall"
                
                if not wall:
                    state = "stop" # After while stop roomba when no wall is found
                
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
                    ds,dth = roomba.sensors.getOdometry("FWD")
                    th +=dth
                    scan = roomba.sensors.lidar.getScan(t_wait) 
                    r_min = roomba.sensors.lidar.getShortestDistance(scan)
		
                    if wall or r_min <= 0.5:
                       roomba.API.drive(int(speed*1000), followRadius)
                    
                    else:
                       roomba.API.drive(int(speed*1000), -followRadius)
                    
                    ds, dth = roomba.sensors.getOdometry("FWD")
                    
                if True in drops:
                    state = "stop"

                else: 
                    if wall and not bumpRight:
                        state = "followwall" 

                    elif not bumpRight and abs(th) >= abs(angle_threshold):
                        state = "findwall"
                
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
                else:
                    state = "findwall"
                    
            elif state == "findwall":                
                tmp1,tmp2 = roomba.sensors.getOdometry('CCW')
                roomba.turnAngle(speed/2, math.radians(160))                 

                tmp1,tmp2 = roomba.sensors.getOdometry('FWD')
                roomba.driveArc(speed, bigRadius, math.pi, True)
                
                if bumpRight or bumpLeft:
                     # Look around till wall if found
                    roomba.API.spin_left()
                    th = 0
                    dth = 0
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
                else:
                     state = "findwall"  
                
                
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
#        time.sleep(2)
        t_wait = 2
        
        t_start = time.time()
        t_end = t_start + 10
        
        while time.time() < t_end:
            scan = roomba.sensors.lidar.getScan(t_wait)
            print len(scan), " LIDAR measurements received. "
            for measurement in scan:
                print measurement 
            if len(scan) > 0:
                r_min = roomba.sensors.lidar.getShortestDistance(scan)
                print "r_min: ", r_min
        
    elif "-control" in flags: 
        roomba = RoombaAPI(port, baudrate)        
        misc.control(roomba)
    
    elif "-off" in flags:
        roomba = RoombaAPI(port, baudrate)   
        roomba.off()
        
    else:
        print "Please specify a valid flag, use -help for more info on possible flags."
