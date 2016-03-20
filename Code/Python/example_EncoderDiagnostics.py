import pyrobotECS
import time

""" Usage information module pyrobotECS

module pyrobotECS contains 4 classes: PyRobotError, SerialCommandInterface, RoombaSensors, Roomba

Class Roomba calls instance of classes SerialCommandInterface and RoombaSensors,
respectively as roomba.sci and roomba.sensors

Functions are generated based on dictionaries with opcodes and their respective names,
and are created on making first object from class roomba.

"""

#Create object rob from class roomba from module pyrobotECS, connected via a given serial port
rob=pyrobotECS.Roomba('/dev/ttyUSB0')

# Robot in control mode, & play a beep
rob.Control()
rob.sci.song(3,1,64,16)
rob.sci.play(3)

# Perform motor run
rob.sci.full()
rob.DriveStraight(200)# drive forward at 200 mm/s

# Print sensory data with timestamp
t_start = time.time()
t_end = time.time()+3 # end time at 3 seconds, counting from this declaration
print 'Time (s)'+'\\t'+'EncoderCountLeft'+'\\t'+'EncoderCountRight'
while time.time() < t_end:
    rob.sci.ser.flushInput()
    time.sleep(0.020)
    r=rob.sensors.getRightEncoderCounts()
    l=rob.sensors.getLeftEncoderCounts()
    time_passed = time.time()- t_start
    print time_passed,'\t',r,'\t',l

# Return to stationary state
rob.DriveStraight(0)
rob.sci.song(3,1,64,16)
rob.sci.play(3)
rob.Control()
    
