import cv2 
import numpy as np
import matplotlib.pyplot as plt 
import time 
import glob


# -------------------- YOLO CLASS OBJECT --------------------
class YoloClass: 
    # Where to get B&W images
    images_path = glob.glob(r"C:/Users/massi/OneDrive/Desktop/TESTING_YOLO_VERSIONS/dataset_files/Capture/Himax/*.bmp")
    
    # Class intantiation 
    def __init__(self, name, yolo_cfg,yolo_weight):
        # Istantiate object constant variables 
        self.name = name
        self.yolo_cfg = yolo_cfg
        self.yolo_weight = yolo_weight
        # Variable objects 
        self.frames_FPS = [] 
        self.frames_detec = []
    
    # Populating the inference time vector 
    def add_frame_fps(self,fps):
        self.frames_FPS.append(fps)

    # Populate the detection vector 
    def add_frame_detect(self, detect): 
        self.frames_detec.append(detect)


# -------------------- DETECTION FUNCTION --------------------
def Detection(yoloClass):

    print('\nCurrently detecting person with: ' + yoloClass.name)

    # Initializing variables 
    classes = ["person"] 
    cuda = True

    if(cuda):
        print('CUDA : ON \n')
    else:
        print('CUDA : OFF \n')
    

    net = cv2.dnn.readNet(yoloClass.yolo_weight,yoloClass.yolo_cfg)
    if cuda:    
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = (255,0,0)

    # Define IR image dimensions
    lep_height = 60
    lep_width = 80
    lep_channels = 3

    # loop through all the B&W images
    for img_path in yoloClass.images_path:

        frameName = str(img_path)[-13:-4]

        img = cv2.imread(img_path)  # open image

        start_time = time.time()
        # Loading image
        if img is None: 
            break
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

        current_bb = []
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y - 5), font, 1, color, 2)

                #Save the bounding box for the IR image
                x_lep = ((x-28))/lep_width          # shift adjusted manually for better results 
                y_lep = ((y-7))/lep_height          # shift adjusted manually for better results 
                lepton_txt_file = open("./Capture/Lepton/" + frameName + ".txt", "w+")
                lepton_txt_file.write("0 " + str(x_lep) + " " + str(y_lep) + " " + str(w/lep_width*0.85) + " " + str(h/lep_height*1))
                # mutiplication of 0.85 for better results (adjussted manually)
                current_bb = [int(x_lep*lep_width), int(y_lep*lep_height), int(w*0.85), h]

               
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

        # Show image on screen and save it
        # Himax
        zoom_img = cv2.resize(img, (0, 0), fx=3, fy=3)
        cv2.imshow("Detection", zoom_img)
        cv2.imwrite("./Capture/Himax_detected/"+frameName+".bmp",  zoom_img)

        # Lepton        
        path_1 = 'C:/Users/massi/OneDrive/Desktop/TESTING_YOLO_VERSIONS/dataset_files/Capture/Lepton/'+frameName+'.bmp'
        tmp_img = cv2.imread(path_1)
        if len(current_bb)>0:   
            a = current_bb[0]  
            b = current_bb[1]       
            c = current_bb[2]
            d = current_bb[3]            
            cv2.rectangle(tmp_img, (a-int(c/2),b-int(d/2)), (a+int(c/2), b+int(d/2)), (255,0,0), 1)
        tmp_img = cv2.resize(tmp_img, (0, 0), fx=3, fy=3)
        cv2.imshow('Lepton detected', tmp_img)


        cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()






# ----------------------------------------------------------
# -------------------------- MAIN --------------------------
# ----------------------------------------------------------


# Generating objects 
yolov3 = YoloClass("Yolo V3" , "./yolov3/yolov3_testing.cfg", "./yolov3/yolov3_training_HIMAX_last.weights")
yolov3_tiny = YoloClass("Yolo V3 tiny", "./yolov3_tiny/yolov3_training.cfg", "./yolov3_tiny/yolov3_training_last.weights")
yolov4 = YoloClass("Yolo V4", "./yolov4/yolov3_training.cfg", "./yolov4/yolov3_training_last.weights")
yolov4_tiny_3l = YoloClass("Yolo V4 tiny 3l", "./yolov4_tiny_3l/yolov3_training.cfg", "./yolov4_tiny_3l/yolov3_training_last.weights")



# Running the detection algorithm for the object of interest
print('')
Detection(yolov3)

print('\n####   Labeling of the Lepton frames completed   #### \n\n')
