# -*- coding: utf-8 -*-
"""
Created on Sun Mar 20 15:10:01 2016

@author: Robin Amsters

Script that simplifies the process of updating files through scp.
This script only works on a UNIX like systems, it was tested on Ubuntu but 
should in theory work on mac (USE AT OWN RISK).

This file is to be run from the terminal and an option should always be
specified.

Example:
    python UpdateCode.py -help

This will list all the available options.

NOTES:
    - To remove files the code should be run from the pi, be carefull to check 
      the prompt so that the right folder is deleted.
      
    - After removing the folder, it is likely that you are inside a directory
      that no longer exists (unless the code was moved this always the case).
      you can escape this by changing your directory to the home 
      (run the command: cd ~)
      
    - When making a direct connection from the pc to the pi via ethernet, enter
      'Direct' when prompted for an IP adress, but make sure the address inside
      the script is correct. Unless many people are interested in this option, 
      the IP adress wil stay hard coded and changed if needed.
"""

from Tkinter import Tk
from tkFileDialog import askopenfilename, askdirectory
import os
import sys

"""
Defining neccesary functions
"""
#==============================================================================
def getFilePath(msg):
    # Selecting file trough GUI
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    filePath = askopenfilename(title = msg) # show an "Open" dialog box and return the path to the selected file
    return filePath
#==============================================================================
def getDirectoryPath(msg):
    # Selecting directory trough GUI
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    directoryPath = askdirectory(title = msg) # show an "Open" dialog box and return the path
    return directoryPath
#==============================================================================
def checkInput(piIpString, groupString):
    # Checking whether input is correct
    
    inputCorrect = False
    piNumber = 0
    groupNumber = 0
    
    # Override for direct connection
    if piIpString == 'Direct':
        inputCorrect = True
        piNumber = 'Direct'
        groupNumber = 1
    
    else:
    
        try:
           piNumber = int(piIpString)
           groupNumber = int(groupString)
           if 0 <= piNumber and piNumber <= len(piIpAdresses)-1 and 1 <= groupNumber and groupNumber <= len(groups):
                inputCorrect = True
           else:
               print inputError
               inputCorrect = False
               
        except ValueError:
           print inputError
           inputCorrect = False
       
    return inputCorrect, piNumber, groupNumber
#==============================================================================    
def updateFile(piIP, localFile, remoteFolder):
    #Updating a single file
    command = 'scp "%s" pi@"%s:%s"' % (localFile, piIP, remoteFolder)
    os.system(command)
#==============================================================================
def updateFolder(piIP, localFolder, remoteFolder):
    #Updating an entire folder
    print " "
    print "Copying: "
    print " "   
    command = 'scp -r "%s" pi@"%s:%s"' % (localFolder, piIP, remoteFolder)
    os.system(command)
    print " "    
    print "Done copying."
#==============================================================================        
def removeFolder(remoteFolder):
    #Removing an entire folder
    print " "
    print "Removing files. "
    print " "
#    remoteFolderFiles = glob.glob(remoteFolder + '/*.py')
#    for remoteFile in remoteFolderFiles:
    command = 'rm -rf "%s"' % (remoteFolder)
    print command
    os.system(command)
    print " "
    print "Done removing files."
#==============================================================================
"""
Updating code using defined functions.
"""
 
if __name__ == "__main__":
    
    # Defining general variables
    orders = sys.argv[1:] #Specified options when running file
    if len(orders) == 0:
       print "Please specify an option to move or delete files, add help as an option for more information."
    
    remoteFileBase = '/home/pi/' #Assume that everyone has a folder inside this directory
    piIpAdresses = {0:'192.168.1.100', 1:'192.168.1.101', 2:'192.168.1.102', 3:'192.168.1.104', 4:'192.168.1.104', 'Direct': '192.168.1.4'} #Valid IP adresses
    groups = {1: 'Robin_Cederic/RoombaLibrary'} #Names of folders of different groups
    # Error messages
    inputError = "Incorrect input."
    
    # First check if user just wants help.
    if "-help" in orders or "help" in orders or "halp" in orders:
        
        print " "
        print "File: ", sys.argv[0]
        print " "
        print "Valid pi numbers are: ", piIpAdresses.keys()
        print "Valid groups are: ", groups
        print " "
        print "Possible options are: "
        print "\t-file: copy a single file to a pi."
        print "\t-folder: copy an entire folder to a pi."
        print "\t-remove: remove an entire folder from a pi (script has to be run from to pi for this option to work)."    
        print " "
    
    elif  len(orders) > 0:
        # Asking user for Roomba number
        piIpString = raw_input("What is the number of the Roomba you are using (please type only 1 number): ")
        groupString = raw_input("What your groupnumber (please type only 1 number): ")
        
           # Script can continue if the input is correct
        inputCorrect, piNumber, groupNumber = checkInput(piIpString, groupString)
        
        if inputCorrect:
            piIP = piIpAdresses[piNumber] #Assigning pi IP address
            group = groups[groupNumber] #Group number
            remoteFolder = remoteFileBase + group #Name of remote folder 
            
            if "-file" in orders:
                localFile = getFilePath("Please select the file that you want to copy to the pi") #Getting local file path
                updateFile(piIP, localFile, remoteFolder)
            
            if "-folder" in orders:
                localFolder = getDirectoryPath("Please select the folder that you want to copy to the pi") #Getting local directory path
                updateFolder(piIP, localFolder, remoteFolder)
                
            if "-remove" in orders:
                sure = raw_input("Are you sure you want to delete all the python files inside: " + remoteFolder + "   (y/n)? ")
                
                if sure == "y":
                    removeFolder(remoteFolder)
                elif sure == "n":
                    print "Files were not deleted."
                else:
                    print inputError

    
