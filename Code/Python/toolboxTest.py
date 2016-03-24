# -*- coding: utf-8 -*-
"""
Created on Thu Mar 24 14:46:02 2016

@author: Robin Amsters

Script to test different aspects of ECSToolbox
"""

import pyrobotECS
import ECSToolbox as et

rob=pyrobotECS.Roomba('/dev/ttyUSB0')
rob.Control()
bumpLeft, bumpRight, bumpFront, dropLeft, dropRight = et.getSafetySensors(rob)