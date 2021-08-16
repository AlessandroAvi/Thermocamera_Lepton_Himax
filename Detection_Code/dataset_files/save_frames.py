import cv2 
import numpy as np
import matplotlib.pyplot as plt 
import time 
import glob
import serial.tools.list_ports
import serial
import copy
import random




# -------------------- READ FROM STM FUNCTIONS --------------------
def readHimax():
    for i in range(himaxHeightSize):  # cycle over height
        pxl = serialInst.read(himaxLineSize) # read one line
        for j in range(himaxLineSize):     # cycle over width
            himax_frame[i][j][0] = pxl[j]   # GREY scale
            himax_frame[i][j][1] = pxl[j]   # GREY scale
            himax_frame[i][j][2] = pxl[j]   # GREY scale


def readLepton():
    for i in range(leptonHeightSize):  # cycle over height
        pxl = serialInst.read(leptonLineSize) # read one line
        for j in range(leptonLineSize):     # cycle over width
            if(j%3 == 0):
                lepton_frame[i][j//3][0] = pxl[j]     # RED
            elif(j%3 == 1):
                lepton_frame[i][j//3][1] = pxl[j]     # GREEN
            elif(j%3 == 2):
                lepton_frame[i][j//3][2] = pxl[j]     # BLUE









# ----------------------------------------------------------
# -------------------------- MAIN --------------------------
# ----------------------------------------------------------


# **********************************************
RECORD_ENABLE = False      # || SAVE FRAME FLAG
# **********************************************


# Create instance of the serial port
ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
# Serial port informations
serialInst.baudrate = 1764705   
serialInst.port = "COM3"

serialInst.open()

# Definition of matrices sizes
himaxLineSize = 164
himaxHeightSize = 122
leptonLineSize = 80*3
leptonHeightSize = 60
mergeLineSize = 164
mergeHeightSize = 122

countFrame_backup = open("last_frame.txt","r+")
countFrame = int(countFrame_backup.readline())
countFrame_backup.close()

# Values for the merging of the images
h_off = 31
h_end = 31+60
w_off = 41
w_end = 41+80



# Matrix containers of the images from cameras
himax_frame  = np.zeros((122,164,3), np.uint8)
lepton_frame = np.zeros((60,80,3), np.uint8)

print("\n PRESS BLUE BUTTON TO START VIDEO STREAM ON LAPTOP \n")

# Begin the infinite while loop for reading the images
while True:

    start_time = time.time()

    # *** read images from the STM throught UART   
    readHimax()
    readLepton()

    # *** create MERGE matrix
    merge_frame  = copy.deepcopy(himax_frame)
    alpha = 0.8
    merge_frame[h_off:h_end, w_off:w_end,:] = cv2.addWeighted(lepton_frame, alpha, himax_frame[h_off:h_end, w_off:w_end,:], 1-alpha, 0.0)

    # *** save the images
    if (RECORD_ENABLE==True):
        if(countFrame < 10):
            name_frame = "000"+str(countFrame)+"frame.bmp"
        elif(countFrame<100):
            name_frame = "00"+str(countFrame)+"frame.bmp"
        elif(countFrame<1000):
            name_frame = "0"+str(countFrame)+"frame.bmp"
        else:
            name_frame = str(countFrame)+"frame.bmp"

        #if cv2.waitKey(1) & 0xFF == ord('q'):

        cv2.imwrite("./Capture/Lepton/"+name_frame, lepton_frame)
        cv2.imwrite("./Capture/Himax/"+name_frame,  himax_frame)
        cv2.imwrite("./Capture/Merge/"+name_frame,  merge_frame)
        countFrame = countFrame+1 
        print(countFrame)


    # display image
    him1 = cv2.resize(himax_frame, (0, 0), fx=4, fy=4)  # Himax
    cv2.imshow('Himax frame', him1)
    lep1 = cv2.resize(lepton_frame, (0, 0), fx=4, fy=4) # Lepton
    cv2.imshow('Lepton frame', lep1)
    #merge1 = cv2.resize(merge_frame, (0, 0), fx=4, fy=4) # Merged
    #cv2.imshow('Merged frame', merge1)

    end_time = time.time() 
    el_time = end_time - start_time
    fps = 1/el_time
    print(el_time)

    cv2.waitKey(1) 


cv2.destroyAllWindows()




