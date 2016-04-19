# -*- coding: utf-8 -*-
"""
Created on Fri Apr 08 11:04:28 2016

@author: u0101246a
"""

import rplidarECS
import time

lidar = rplidarECS.RPLidar("COM4")

lidar.connect()

lidar.get_health()

lidar.start_monitor()
time.sleep(0.5)
data = lidar.scan_data()

lidar.disconnect()

for i in range(0,len(data)):
    print data[i]

time.sleep(5)
lidar.init_xy_plot()
lidar.update_xy_plot()

#t=time.time()
#
#while True:
#    try:
#        if t+3 < time.time():
#            lidar.update_xy_plot()
#            t=time.time()
#    except KeyboardInterrupt:
#        print "CTRL-c pressed, exiting..."
#        lidar.disconnect()
#        lidar.stop_monitor()    
#        pass
