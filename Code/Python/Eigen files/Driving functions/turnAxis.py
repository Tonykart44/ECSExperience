def turnAxis(robot,velocity,angle,direction):

    sensors = robot.sensors.getBumpsandWheelDrops() #dict with sensors
    roombaStopped = 0 #Roomba is stopped or driving
    s = 0 #distance driven
    th = 0 #angle turned

    robot.TurnInPlace(velocity,angle,direction) #set velocity

    while th < abs(angle) and not roombaStopped:
        if 1 in sensors.itervalues(): 
            robot.Stop()
            roombaStopped = 1
        
        ds, dth = et.getOdometry(robot)
        
        s = s + ds
        th = th + dth
        
    robot.TurnInPlace(0,0,direction)
