def driveStraight(robot,velocity,distance):

    sensors = robot.sensors.getBumpsandWheelDrops() #dict with sensors
    roombaStopped = 0 #Roomba is stopped or driving
    s = 0 #distance driven
    th = 0 #angle turned

    robot.DriveStraight(velocity) #set velocity

    while s < abs(distance) and not roombaStopped:
        if 1 in sensors.itervalues(): 
            robot.Stop()
            roombaStopped = 1
        
        ds, dth = et.getOdometry(robot)
        
        s = s + ds
        th = th + dth
        
    robot.DriveStraight(0)
    
