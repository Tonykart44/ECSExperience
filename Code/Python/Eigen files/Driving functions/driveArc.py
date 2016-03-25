def driveArc(robot,velocity,radius,angle):

    sensors = robot.sensors.getBumpsandWheelDrops() #dict with sensors
    roombaStopped = 0 #Roomba is stopped or driving
    s = 0 #distance driven
    th = 0 #angle turned

    robot.Drive(velocity,radius) #set velocity

    while th < abs(angle) and not roombaStopped:
        if 1 in sensors.itervalues(): 
            robot.Stop()
            roombaStopped = 1
        
        ds, dth = et.getOdometry(robot)
        
        s = s + ds
        th = th + dth
        
    robot.Drive(0,0)
    