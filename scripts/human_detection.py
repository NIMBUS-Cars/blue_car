#!/usr/bin/env python
#!/usr/bin/env python3

import sys
from numpy import NaN
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 as pc2
import tf, tf2_ros
from geometry_msgs.msg import Point, PointStamped
import csv
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import math
import time 
# from pathlib import Path
from testrobots.msg import Boundingbox
from testrobots.msg import Plot
from timeit import default_timer as timer
from datetime import timedelta
# from memory_profiler import profile



bridge = CvBridge() 
class Detection(object):
    
    
    def __init__(self):
        self.corner_queue = []
        self.queue_center = []
        
        self.center_pixel = [] 
        self.corners = 0   # list containing lists of corners for current timestamp - recieved from 
        
        self.center_depth_point = Point()
        self.depth = 0
        self.confidence = 0.0
        
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback,queue_size=1)
     
        
        # publishing topics
        self.yolo_out = rospy.Publisher("Yolo_image", Image, queue_size=1)    
        self.human_msg = rospy.Publisher("H_Detection_msg", Plot, queue_size=1)
        self.stop_msg =  rospy.Publisher("Stop_msg", Plot, queue_size=1)
        self.vector_pub = rospy.Publisher("H_Vector", Image, queue_size=1)      
        self.boundingbox = rospy.Publisher("BBox", Boundingbox, queue_size=1)
        self.depth_with_BB = rospy.Publisher("DepthBB", Image, queue_size=100, latch=True)
        
        #initialize csv file
        self.path = os.getcwd()
        os.chdir("../")
        self.path2 = os.getcwd()       
        self.path3 = os.getcwd()
        self.path4 = os.getcwd()
        self.path = self.path+"/human_motion.csv"
        # /testrobot/test_catkin/src/testrobots
        self.absolutepath_cfg = self.path2 + "/scripts/yolov3.cfg"
        self.absolutepath_weights = self.path3 + "/scripts/yolov3.weights"
        self.absolutepath_coco_names = self.path4 + "/scripts/coco.names"
        
        # open the file in the write mode        
        header = ['center_x', 'center_y', 'Distance']
        self.csv_file = open(self.path, 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(header)
    
    
    def image_callback(self,data):
        start = timer()
        print("image callback")
        cv_img =  bridge.imgmsg_to_cv2(data)
        self.yolo_processing(cv_img)
        end = timer()
        print(timedelta(seconds=end-start)) #in seconds
        print()
        
    
    def yolo_processing(self,cv_img):       
        ''' 
        yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
        '''
        # defining msgs for publishing
        msg = Plot()
        msg.value = -1
       
        bbcordmsg = Boundingbox()
       
  
        yolo_output, object_label, center_pixels, self.corners, self.confidence = self.yolo_imp(cv_img)
        
        # checking center_pixels and setting center_pixel to 0         
        if len(center_pixels) == 0: 
            self.center_pixel = []
            print("no center pixel in yolo_processing...",self.center_pixel)
        else:
            self.center_pixel = center_pixels[0]
            

        #making the yolo output into a ros image version        
        output = bridge.cv2_to_imgmsg(yolo_output)
       
        # changing the msg value only if the label is == person
        if(object_label == 'person'):
            rospy.logwarn("Human Detected on Camera")
            
            # open the corner pixel and get the xmin,xmax, ymin, max this is only done and published when there is a human   
            
            corner = self.corners[0]
            leftbottom_corner = corner[0]
            rightbottom_corner = corner[1]
            lefttop_corner = corner[2]
            righttop_corner = corner[3]  
            
            xmin = leftbottom_corner[0]
            xmax = righttop_corner[0]
            ymin = leftbottom_corner[1]
            ymax = righttop_corner[1]
            
            #complete the bbcord message now
           
            bbcordmsg.Class = object_label
            bbcordmsg.probability = float(self.confidence)
            bbcordmsg.xmin = xmin
            bbcordmsg.xmax = xmax
            bbcordmsg.ymin = ymin
            bbcordmsg.ymax = ymax
       
            msg.value = 1 
        else:
            rospy.logwarn("No Human")
      
        
        #publish the message and the image
        self.human_msg.publish(msg)
        print("h_detect",msg.value)
        self.yolo_out.publish(output)
        self.boundingbox.publish(bbcordmsg)
        
        # checking if center_pixels is empty and then setting the past center
        if len(self.center_pixel) == 0: pass
        else:
            self.queue_center.append(self.center_pixel)
            self.corner_queue.append(self.corners[0])
            
     
    def yolo_imp(self,img_data): 
        start_time = time.perf_counter ()
        corners = []
        
        net = cv2.dnn.readNet(self.absolutepath_cfg,self.absolutepath_weights)
        classes = []

        with open(self.absolutepath_coco_names, 'r') as f:
            
            classes = f.read().splitlines()

      
        height,width,_ = img_data.shape
        
        blob = cv2.dnn.blobFromImage(img_data, 1/255, (256, 256), (0,0,0), swapRB=False, crop=False)


        net.setInput(blob)

        output_layers_names = net.getUnconnectedOutLayersNames()

        layerOutputs = net.forward(output_layers_names)

        boxes = []
        confidences = []
        class_ids = []
        center_pixels = []
        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
                '''
                    scale the bounding box coordinates back relative to the
                    size of the image, keeping in mind that YOLO actually
                    returns the center (x, y)-coordinates of the bounding
                    box followed by the boxes' width and height
                '''
                if confidence > 0.5:                                
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)
                    '''
                        use the center (x, y)-coordinates to derive the top and left corner of the bounding box
                    '''
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    boxes.append([x,y,w,h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    # print(class_id)
                    # if class_id == 0:
                        # center_pixels.append([center_x,center_y])
        print("---------------------------------------------------")
       
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        object_label = ""
        
        font = cv2.FONT_HERSHEY_PLAIN
        colors = np.random.uniform(0, 255, size=(len(boxes), 3))
        if len(indexes)>0:

            for i in indexes.flatten():
                x,y,w,h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = str(round(confidences[i], 2))
                # getting the coordinates +ve
                x = abs(x)
                y = abs(y)
                area = 0
                print("")              
                print("label -",label,
                ", confidence", confidence,
                ", area of Bounding Box  - ",w*h)

                color = colors[i]
                
                leftbottom_corner = [x,y]
                rightbottom_corner = [x+w,y]
                lefttop_corner = [x,y+h]
                righttop_corner =  [x+w,y+h]

                '''
                    Add code to find the center from the coordinates
                '''    
                probable_center = (center_x, center_y)

                center_pixels.append([center_x,center_y])
                corners.append([leftbottom_corner,rightbottom_corner,lefttop_corner,righttop_corner])
                
                cv2.rectangle(img_data,(x,y), (x+w, y+h), color, 2)
                cv2.putText(img_data, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)

        try:
            object_label = label
        except UnboundLocalError: 
            print("no label")

        cv2.imwrite('yolo_img.jpeg', img_data)
        return img_data, object_label, center_pixels,  corners, confidence     
    
      
def main(): 
     
    rospy.init_node('Human_Detection', anonymous=False)    
    sn = Detection()    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main() 