import pyrobotECS
import time

rob=pyrobotECS.Roomba('/dev/ttyUSB0')

# Robot in control mode, & play a beep
rob.Control()
rob.sci.song(3,1,64,16)
rob.sci.play(3)

rob.sci.full()
t_end = time.time()+5
while time.time() < t_end:
    BumpFront = rob.sensors.getBumpsandWheelDrops()
    print BumpFront['bump-right']
