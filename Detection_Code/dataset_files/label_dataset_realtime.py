import cv2 
import numpy as np
import matplotlib.pyplot as plt 
import time 
import glob
import serial.tools.list_ports
import serial
import copy
import random


# -------------------- YOLO CLASS OBJECT --------------------
class YoloClass: 
    # Common class variable 
    images_path = glob.glob(r"C:/Users/massi/OneDrive/Desktop/TESTING_YOLO_VERSIONS/dataset_files/Images/Himax/*.bmp")
    
    # Class intantiation 
    def __init__(self, name, yolo_cfg,yolo_weight):
        # Istantiate object constant variables 
        self.name = name
        self.yolo_cfg = yolo_cfg
        self.yolo_weight = yolo_weight
        # Variable objects 
        self.frames_FPS = [] 
        self.frames_detec = []
        # Cuda enabled
        self.cuda = True
        # Other
        self.colors = (255,0,0)
        self.classes = []
        self.net = []
        self.layer_names = []
        self.output_layers = []
    
    # Populating the inference time vector 
    def add_frame_fps(self,fps):
        self.frames_FPS.append(fps)

    # Populate the detection vector 
    def add_frame_detect(self, detect): 
        self.frames_detec.append(detect)


# -------------------- DETECTION FUNCTION --------------------
def YoloNet_init(yoloClass):
    print('Currently evaluating : ' + yoloClass.name)

    if(yoloClass.cuda):
        print('CUDA : ON \n\n')
    else:
        print('CUDA : OFF \n\n')

    yoloClass.net = cv2.dnn.readNet(yoloClass.yolo_weight,yoloClass.yolo_cfg)
    if yoloClass.cuda:    
        yoloClass.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        yoloClass.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    yoloClass.classes = ["person"]
    yoloClass.layer_names = yoloClass.net.getLayerNames()
    yoloClass.output_layers = [yoloClass.layer_names[i[0] - 1] for i in yoloClass.net.getUnconnectedOutLayers()]


def Detection(yoloClass, himax_frame):

    img = himax_frame

    start_time = time.time()
    # Loading image
    if img is None: 
        print("Error: no image given")
        return
    height, width, channels = img.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    yoloClass.net.setInput(blob)
    outs = yoloClass.net.forward(yoloClass.output_layers)

    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    new_detection = False 
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            print('confidence:' + str(confidence), end='\r')
            if confidence > 0.2:
                new_detection = True
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

                #*** save the lepton bounding box
                if (RECORD_ENABLE==True):
                    if(countFrame < 10):
                        text_name = "000"+str(countFrame)+"frame.txt"
                    elif(countFrame<100):
                        text_name = "00"+str(countFrame)+"frame.txt"
                    elif(countFrame<1000):
                        text_name = "0"+str(countFrame)+"frame.txt"
                    else:
                        text_name = str(countFrame)+"frame.txt"

                    x_lep = ((center_x-42))/80
                    y_lep = ((center_y-31))/60
                    lepton_txt_file = open("./Capture/Lepton/" + text_name, "w+")
                    lepton_txt_file.write("0 " + str(x_lep) + " " + str(y_lep) + " " + str(detection[2]*width/80) + " " + str(detection[3]*height/60))

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(yoloClass.classes[class_ids[i]])
            color = yoloClass.colors[class_ids[i]]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, label, (x, y + 30), font, 3, color, 2)


    end_time = time.time() 
    el_time = end_time - start_time
    fps = 1/el_time
    print('FPS ' + str(fps))
    # Save inferencing time in the class vector ! 
    yoloClass.add_frame_fps(fps)
    # Save detection in the class vector ! 
    if new_detection : 
        yoloClass.add_frame_detect(fps)
    else :
        yoloClass.add_frame_detect(None)
    zoom_img = cv2.resize(img, (0, 0), fx=3, fy=3)
    cv2.imshow("DETECTION", zoom_img)
    if RECORD_ENABLE:
        cv2.imwrite("./Capture/Himax_detected/"+name_frame,  zoom_img)



# -------------------- PLOTTING FUNCTION --------------------
def PlottingDetectionPoints(vecYoloClasses):
    print(" Plotting results ")
    colors = ["red", "blue", "green"]
    i = 0
    for yoloClass in vecYoloClasses: 
        avg_time = np.average(yoloClass.frames_FPS)
        print(f"Avg. fps inference : {avg_time}")

        plt.title("Inferencing time")
        plt.xlabel = "N. frames"
        plt.ylabel = "Time [s]"
        plt.scatter(range(0,len(yoloClass.frames_FPS)),yoloClass.frames_FPS,facecolors='none',edgecolors=colors[i])
        plt.scatter(range(0,len(yoloClass.frames_FPS)),yoloClass.frames_detec,color=colors[i])
        i += 1 
    plt.show()


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


# Generating YOLO objects 
yolov3 = YoloClass("Yolo V3" , "./yolov3/yolov3_testing.cfg", "./yolov3/yolov3_training_HIMAX_last.weights")
yolov3_tiny = YoloClass("Yolo V3 tiny", "./yolov3_tiny/yolov3_training.cfg", "./yolov3_tiny/yolov3_training_last.weights")
yolov4 = YoloClass("Yolo V4", "./yolov4/yolov3_training.cfg", "./yolov4/yolov3_training_last.weights")
yolov4_tiny_3l = YoloClass("Yolo V4 tiny 3l", "./yolov4_tiny_3l/yolov3_training.cfg", "./yolov4_tiny_3l/yolov3_training_last.weights")

RECOGNIZE_ENABLE = True
RECORD_ENABLE = True

# Create instance of the serial port
ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
# Serial port informations
serialInst.baudrate = 1764705   
serialInst.port = "COM3"

serialInst.open()

# Definizion of matrices sizes
himaxLineSize = 164
himaxHeightSize = 122
leptonLineSize = 80*3
leptonHeightSize = 60
mergeLineSize = 164
mergeHeightSize = 122
countFrame = 0


# Matrix containers of the images from cameras
himax_frame  = np.zeros((122,164,3), np.uint8)
lepton_frame = np.zeros((60,80,3), np.uint8)


# Initialize the YOLO4 net
if RECOGNIZE_ENABLE:
    YoloNet_init(yolov4)

print("\n PRESS BLUE BUTTON TO START VIDEO STREAM ON LAPTOP \n")

# Begin the infinite while loop for reading the images
while True:

    # *** read images from the STM throught UART   
    readHimax()
    readLepton()

    # *** create MERGE matrix
    merge_frame  = copy.deepcopy(himax_frame)
    # ????  DA FINIRE

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

        #cv2.imwrite("./Capture/Lepton/"+name_frame, lepton_frame)
        #cv2.imwrite("./Capture/Himax/"+name_frame,  himax_frame)
        #cv2.imwrite("./Capture/Merge/"+name_frame,  merge_frame)
        countFrame = countFrame+1 
        print(countFrame)

    # Run YOLO4 DETECTION
    if RECOGNIZE_ENABLE:
        Detection(yolov4, himax_frame)

    # display image
    if not RECOGNIZE_ENABLE:
        him1 = cv2.resize(himax_frame, (0, 0), fx=3, fy=3)  # Himax
        cv2.imshow('Himax frame', him1)
    lep1 = cv2.resize(lepton_frame, (0, 0), fx=3, fy=3) # Lepton
    cv2.imshow('Lepton frame', lep1)

    cv2.waitKey(1) 

    # *** press Q to stop script
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break




cv2.destroyAllWindows()



























