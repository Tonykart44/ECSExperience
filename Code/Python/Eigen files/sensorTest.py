# -*- coding: utf-8 -*-
"""
Created on Tue Mar 22 14:20:56 2016

@author: robin
"""

import pyrobotECS
import time

rob=pyrobotECS.Roomba('/dev/ttyUSB0')

# Robot in control mode, & play a beep
rob.Control()
rob.sci.song(3,1,64,16)
rob.sci.play(3)

rob.sci.full()

def getSafetySensors(robot):
    
    sensors = robot.sensors.getBumpsandWheelDrops()
    
    bumpLeft= sensors['bump-left']
    bumpRight= sensors['bump-right']
    bumpFront = bumpLeft and bumpRight
    dropLeft= sensors['wheel-drop-left']
    dropRight= sensors['wheel-drop-right']
    
    return bumpLeft, bumpRight, bumpFront, dropLeft, dropRight
    
t_end = time.time()+5
while time.time() < t_end:
    bumpLeft, bumpRight, bumpFront, dropLeft, dropRight = getSafetySensors(rob)
    
    print "wall sensor: ", rob.sensors.getWall()