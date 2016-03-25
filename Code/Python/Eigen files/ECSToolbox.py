# -*- coding: utf-8 -*-
"""
Created on Tue Mar 22 15:39:12 2016

@author: Robin Amstes
"""

def getOdometry(robot):
    pass
    

def getSafetySensors(robot):
    """
        Specifies the state of the bump and wheel drop sensors, either 
        triggered or not triggered.    
        
        This function is more or less a Python equivalent of the MATLAB 
        function with the same name in the roombaCommuncationToolbox.
        
        INPUTS:
                robot = an object of the class Roomba (see pyrobotECS)
        
        OUTPUTS:
                bumpLeft = value of the left bump sensor, True if the sensor 
                            is pressed, False if it is not pressed.
                           
                bumpRight = value of the right bump sensor, True if the sensor 
                            is pressed, False if it is not pressed 
                           
                bumpfront = value of the front bump sensor, True if the sensor 
                            is pressed, False if it is not pressed.
                           
                dropLeft = value of the left wheel dropsensor, True if the
                           wheel is dropped, False if the wheel is not dropped.
                           
                dropRight = value of the right wheel dropsensor, True if the
                           wheel is dropped, False if the wheel is not dropped.
    
    """
    
    #TODO: check if robot is instance of class Roomba
    
    sensors = robot.sensors.getBumpsandWheelDrops()
    
    bumpLeft= sensors['bump-left']
    bumpRight= sensors['bump-right']
    bumpFront = bumpLeft and bumpRight
    dropLeft= sensors['wheel-drop-left']
    dropRight= sensors['wheel-drop-right']
    
    return bumpLeft, bumpRight, bumpFront, dropLeft, dropRight