import cv2 
import numpy as np
import matplotlib.pyplot as plt 
import time 

# -------------------- YOLO CLASS OBJECT --------------------
class YoloClass: 
    # Common class variable 
    video_bench = "../Videos/Soggiorno3/Himax.mp4"

    # Class intantiation 
    def __init__(self, name, yolo_cfg,yolo_weight, Cuda_FLAG, color):
        # Istantiate object constant variables 
        self.name = name
        self.yolo_cfg = yolo_cfg
        self.yolo_weight = yolo_weight
        # Variable objects 
        self.frames_FPS = [] 
        self.frames_detec = []
        self.Cuda = Cuda_FLAG
        self.plot_color = color
    
    # Populating the inference time vector 
    def add_frame_fps(self,fps):
        self.frames_FPS.append(fps)

    # Populate the detection vector 
    def add_frame_detect(self, detect): 
        self.frames_detec.append(detect)


# -------------------- DETECTION FUNCTION --------------------
def Detection(yoloClass):

    print('Currently evaluating : ' + yoloClass.name)

    # Initializing variables 
    classes = ["person"] 
    

    #net = cv2.dnn.readNetFromDarknet(yoloClass.yolo_cfg, yoloClass.yolo_weight)
    net = cv2.dnn.readNet(yoloClass.yolo_weight,yoloClass.yolo_cfg)
    if yoloClass.Cuda:    
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        print("Cuda: ON")
    else:
        print("Cuda: OFF")

    cap = cv2.VideoCapture(yoloClass.video_bench)

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    #colors = np.random.uniform(0, 255, size=(len(classes), 3))
    colors = (255,0,0)

    ciccio = 0

    # loop through all the images
    while True:
        print(ciccio)
        ciccio = ciccio +1
        start_time = time.time()
        # Loading image
        ret,img = cap.read()
        if img is None: 
            break
        #img = cv2.resize(img, None, fx=3, fy=3)
        height, width, channels = img.shape

        # Detecting objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        net.setInput(blob)
        outs = net.forward(output_layers)

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
                if confidence > 0.2:
                    new_detection = True
                    #print(confidence)
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
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y -5), font, 1, color, 2)

        end_time = time.time() 
        el_time = end_time - start_time
        fps = 1/el_time
        # Save inferencing time in the class vector ! 
        yoloClass.add_frame_fps(fps)
        # Save detection in the class vector ! 
        if new_detection : 
            yoloClass.add_frame_detect(fps)
        else :
            yoloClass.add_frame_detect(None)
        #print(f'Elapsed time : {el_time}')

        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


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
        plt.grid(True, both, both)
        i += 1 
    plt.show()

def BenchmarkAverage(vecYoloClasses):
    print(" Plotting benchmarks ")
    i = 0
    for yoloClass in vecYoloClasses: 
        n_detections = 0        
        avg_fps = np.average(yoloClass.frames_FPS)
        min_fps = np.min(yoloClass.frames_FPS)
        max_fps = np.max(yoloClass.frames_FPS)
        for detect in yoloClass.frames_detec :
            if detect : n_detections += 1

        plt.title("Benchmark plot")
        plt.xlabel("FPS")
        plt.ylabel("N. detections")
        plt.grid(True, linewidth='1', linestyle='--')
        #plt.errorbar(avg_fps,n_detections, xerr=[[avg_fps-min_fps], [max_fps-avg_fps]], fmt='o')
        if yoloClass.Cuda == True:
            plt.scatter(avg_fps,n_detections, marker='o', color=yoloClass.plot_color)
        else:
            plt.scatter(avg_fps,n_detections, marker='D', color=yoloClass.plot_color)    

        i += 1 
    plt.legend([y.name for y in vecYoloClasses])
    plt.show()





################################################################
#                           MAIN
################################################################
#               blue       orange     green      red        violet     brown      pink       gray       yellow     cyan
plot_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
# Generating objects 
# CUDA enabled
yolov3_ON           = YoloClass("Yolo V3 - Cuda ON" , "./yolov3/yolov3_testing.cfg", "./yolov3/yolov3_training_HIMAX_last.weights", True, plot_colors[0])
yolov3_tiny_ON      = YoloClass("Yolo V3 tiny - Cuda ON", "./yolov3_tiny/yolov3_training.cfg", "./yolov3_tiny/yolov3_training_last.weights", True, plot_colors[1])
yolov4_ON           = YoloClass("Yolo V4 - Cuda ON", "./yolov4/yolov3_training.cfg", "./yolov4/yolov3_training_last.weights", True, plot_colors[2])
yolov4_tiny_3l_ON   = YoloClass("Yolo V4 tiny 3l - Cuda ON", "./yolov4_tiny_3l/yolov3_training.cfg", "./yolov4_tiny_3l/yolov3_training_last.weights", True,plot_colors[4])
# CUDA disabled
yolov3_OFF          = YoloClass("Yolo V3 - Cuda OFF" , "./yolov3/yolov3_testing.cfg", "./yolov3/yolov3_training_HIMAX_last.weights", False, plot_colors[0])
yolov3_tiny_OFF     = YoloClass("Yolo V3 tiny - Cuda OFF", "./yolov3_tiny/yolov3_training.cfg", "./yolov3_tiny/yolov3_training_last.weights", False, plot_colors[1])
yolov4_OFF          = YoloClass("Yolo V4 - Cuda OFF", "./yolov4/yolov3_training.cfg", "./yolov4/yolov3_training_last.weights", False, plot_colors[2])
yolov4_tiny_3l_OFF  = YoloClass("Yolo V4 tiny 3l - Cuda OFF", "./yolov4_tiny_3l/yolov3_training.cfg", "./yolov4_tiny_3l/yolov3_training_last.weights", False, plot_colors[4])

# Populating vector of objects 
classesVec = [] 
classesVec.append(yolov3_ON)
classesVec.append(yolov3_tiny_ON)
classesVec.append(yolov4_ON)
classesVec.append(yolov4_tiny_3l_ON)

classesVec.append(yolov3_OFF)
classesVec.append(yolov3_tiny_OFF)
classesVec.append(yolov4_OFF)
classesVec.append(yolov4_tiny_3l_OFF)

# Running the detection algorithm for every object 
for yolo in classesVec: 
    Detection(yolo)

# Plot Benchmark 
BenchmarkAverage(classesVec)




























