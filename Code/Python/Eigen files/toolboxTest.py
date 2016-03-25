# -*- coding: utf-8 -*-
"""
Created on Thu Mar 24 14:46:02 2016

@author: Robin Amsters

Script to test different aspects of ECSToolbox
"""

import pyrobotECS
import ECSToolbox as et
import time

rob=pyrobotECS.Roomba('/dev/ttyUSB0')
rob.Control()
rob.sci.song(3,1,64,16)
rob.sci.play(3)

t_start = time.time()
t_end = time.time()+30 # end time at 5 seconds, counting from this declaration


rob.TurnInPlace(200, 'cw')
angle_low = rob.sensors.getRightEncoderCounts()
angle_high = rob.sensors.getLeftEncoderCounts()

print "Encoders"
print angle_low
print angle_high

while time.time() < t_end:
    
    rob.sci.ser.flushInput()
    time.sleep(0.020)
    print "Encoders"
    angle_low = rob.sensors.getRightEncoderCounts()
    angle_high = rob.sensors.getLeftEncoderCounts()
    print angle_low
    print angle_high

#    rob.sensors.Angle(angle_low, angle_high, 'degrees')
    

rob.Stop()

print rob.sensors.data
