# Thermocamera_Lepton_Himax

This repository contains the code developed for the project course Embedded Systems - University of Trento

The project is also explained in [this paper](https://openaccess.thecvf.com/content/ICCV2021W/DSC/html/Avi_Infrared_Dataset_Generation_for_People_Detection_Through_Superimposition_of_Different_ICCVW_2021_paper.html). 

# Project's goal 
Our project's goal was to :

- develop and build a thermal scanner prototype based on an STM32 microcontroller. The developed code allows the user to fuse the images coming from two different cameras (visual and thermal), as well as visualizing the end result thanks to the open source library OpenCV.  
- crate an automatic labeler for the Infrared images based on the human detection performed on black and white frames. The detection is performed with YOLO.

# Hardware 
The hardware components that have been used for this project are : 
* STM32 F401RE 
* Himax HM01B0 visual camera 
* Lepton Flir v2.0 infra-red camera 

# Code 
The code is mainly splitted into 2 folders:
* `Detection_Code`
   - Python scripts used for seeing in real time the video stream from the microcontroller and save the frames
   - Python scripts used for performing a real time detection and labeling on the incoming stream
   -  Python scripts used for performin an off line detection and labeling based on a video
* `STM_Code`
   - STM code developed in the CubeIDE environment for reading the data captured by the two cameras. 
   - Further distinction is made for code which is generic-purpose and code which is specific for a single camera  

# Dataset

This repo also contains the dataset that has been created with the detection and labeling method, both the infrared and black and white images can be found here. Specifically in the folder 2 files can be found:

- `Dataset.zip` which contains all the 3655 couples of IR and B&W images
- `Dataset_100_sample.zip` which contains only 100 couples of IR and B&W images as a sample

![name-of-you-image](https://github.com/AlessandroAvi/Thermocamera_Lepton_Himax/blob/main/Images/dataset_example.png)



![name-of-you-image](https://github.com/AlessandroAvi/Thermocamera_Lepton_Himax/blob/main/Images/dataset_example_himax.png)

# Results 
Video footage of the working prototype can be found at the following link : 
[Experimental video footage](https://www.youtube.com/watch?v=Bs92HlgQElU)

Mockup still images can be found in the folder `./images/` of this repository. 

# Mentions 
The code for the Lepton library has been highly inspired by the work of NachtRaveVL (https://github.com/NachtRaveVL/Lepton-FLiR-Arduino).

