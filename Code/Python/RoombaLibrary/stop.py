# -*- coding: utf-8 -*-
"""
Created on Tue Mar 15 16:47:18 2016

@author: Robin Amsters
"""

import pyrobotECS

try: 
    rob=pyrobotECS.Roomba('/dev/ttyUSB0')
    rob.Stop()
    
except:
    rob=pyrobotECS.Roomba('/dev/ttyUSB1')
    rob.Stop()
    
try: 
    rob=pyrobotECS.Roomba('/dev/ttyUSB1')
    rob.Stop()
    
except:
    rob=pyrobotECS.Roomba('/dev/ttyUSB0')
    rob.Stop()