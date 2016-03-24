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
t_end = time.time()+10 # end time at 10 seconds, counting from this declaration
angle_low = bytes(0)
angle_high = bytes(0)

while time.time() < t_end:
    rob.sensors.Angle(angle_low, angle_high, 'radians')
    print rob.sensors['angle']
    print rob.sensors['distance']