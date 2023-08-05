#!/usr/bin/env python

                            
                            #  Project Name: 	autonomus_weed_detection_and_removal_system
                            #  Author List: 		Tushar Kurne
                            #  Filename: 		weed_detection_final.py
                            #  Functions: 		image_detection
                            #  Global Variables:	MODEL_NAME,MODEL_PATH,PATH_TO_CKPT,LABEL_NAME,PATH_TO_LABELS,NUM_CLASSES,count

import rospy
import cv2
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64MultiArray
import sys
import getch
import time
import sys,select
import os
import numpy as np  
import tensorflow as tf 
from std_msgs.msg import String , Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from rospy.numpy_msg import numpy_msg
import math

import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

MODEL_NAME =  'inference_graph_from_colab_rcnn_new'  #name of the object detection model used
MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','models' , MODEL_NAME) #path of folders data and models
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb' #path to graph.pb file
LABEL_NAME = 'labelmap.pbtxt' #name of .pbtxt file
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME) #path to labels 
NUM_CLASSES = 3 #number of classes model able to detect

bridge=CvBridge()
count=1 #used to count number of captured images

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
sess = tf.Session(graph=detection_graph)


                            
                            
                            # Function Name: 	image_detection
                            # Input: 		accepts data of type Image published by usb_cam
                            # Output: 		publishes processed image, coordinates of weed
                            # Logic: 		It captures the image after accepting input from user processes it using tensorflow graph. Finds out the bounding box coordinates and centre of them and  						publishes further information 
                            # Example Call:		callback function hence no need to call individuallly
                            
                            
					
def image_detection(data):
        coordinate_pub=rospy.Publisher("weed_pixels",numpy_msg(Floats),queue_size=1)
        mid_point_pub=rospy.Publisher("mid_point",Point,queue_size=10)
        image_pub=rospy.Publisher("output_image",Image,queue_size=1)
        mov_fwd=rospy.Publisher("go_ahead",Int8,queue_size=1)
        i, o, e = select.select( [sys.stdin], [], [], 0.01 )
        k=9 #used to compare keystroke valueenterd from user
        

        if(i):
            i=sys.stdin.readline().strip() #accepts input from keyboard
            print('processing')
            if(k==int(i)):
                print(int(i))
                cv_image=bridge.imgmsg_to_cv2(data,"bgr8") #converts message of type image form opencv images to ros image
                global count
                cv2.imwrite("/home/nuc/catkin_ws/src/weed_detection_with_tensorflow/captured/image_{}.jpg".format(count),cv_image)
                image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB) #stores image after color conversion

                image_np=np.asarray(image) #image is converted into numpy array
                image_np_expanded = np.expand_dims(image_np, axis=0) #expanding dimensions of the image
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                scores = detection_graph.get_tensor_by_name('detection_scores:0')
                classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                
                objects=vis_util.visualize_boxes_and_labels_on_image_array(
                    image,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=2,
                    min_score_thresh=0.9)

                img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB) 
                cv2.waitKey(1) 
                cv2.imshow("processed_image",img)
                cv2.imwrite("/home/nuc/catkin_ws/src/weed_detection_with_tensorflow/processed/output_{}.jpg".format(count),img)
                
                bboxes=boxes[scores>0.9] #bounding boxes with prediciton score higher than 0.9
                height,width=image.shape[:2] #to store height and width of the image in terms of pixel
                final_box = [] #stores corner points of bounding box coordinates 
                t=classes[scores>0.9] #classes with prediciton score higher than 0.9
                bboxes=bboxes[t==1] # bounding boxes belonging to classes with scores higher than 0.9
                for box in bboxes:
                    ymin, xmin, ymax, xmax = box
                    ymin=0-ymin #making y coordinates negative
                    ymax=0-ymax
                    final_box = [int(ymin * height), int(xmin * width), int(ymax * height), int(xmax * width)] #list of coordinates in terms of pixels
                    final_box_mm = [0.552*int(ymin * height), 0.552*int(xmin * width), 0.552*int(ymax * height), 0.552*int(xmax * width)] #list of coordinates in terms of distance in millimeter
                    lucor=np.asarray([final_box[1],final_box[0]]) #upper left corner point in pixels
                    ldcor=np.asarray([final_box[1],final_box[2]]) #down left corner point in pixels
                    rucor=np.asarray([final_box[3],final_box[0]]) #upper right corner point in pixels
                    rdcor=np.asarray([final_box[3],final_box[2]]) #down right corner point in pixels
                    f=np.array([lucor,ldcor,rucor,rdcor],dtype=np.float32) #list is converted into an array
                    lucor_mm=np.asarray([final_box_mm[1],final_box_mm[0]]) #upper left corner point in mm
                    ldcor_mm=np.asarray([final_box_mm[1],final_box_mm[2]]) #down left corner point in mm
                    rucor_mm=np.asarray([final_box_mm[3],final_box_mm[0]]) #upper right corner point in mm
                    rdcor_mm=np.asarray([final_box_mm[3],final_box_mm[2]]) #down right corner point in mm
                    f_mm=np.array([lucor_mm,ldcor_mm,rucor_mm,rdcor_mm],dtype=np.float32) #list is converted into an array
                    coordinate_pub.publish(f_mm) #publishes coordinates in mm
                    w=abs(final_box[3]-final_box[1])/2 #width of bounding box
                    h=-abs(final_box[2]-final_box[0])/2 #height of bounding box
                    w_mm=abs(final_box_mm[3]-final_box_mm[1])/2 #width in mm
                    h_mm=-abs(final_box_mm[2]-final_box_mm[0])/2 #height in mm
                    mid=Point()
                    mid.x=w_mm+final_box_mm[1] #x coordinate of centre point
                    mid.y=h_mm+final_box_mm[0] #y coordinate of centre point
                    mid.z=-340 #z cordinate is kept constant 
                    print(mid)
                    mid_point_pub.publish(mid) #publishing centre point 
                    file1=open("/home/nuc/catkin_ws/src/weed_detection_with_tensorflow/weed_coordinates.txt","a") #to write down coordinates in text file
                    file1.write("image_{}  ".format(count)+str(mid2))
                    file1.write("\n")
                    file1.close()
                image_out = bridge.cv2_to_imgmsg(img,"bgr8")
                image_out.header = data.header
                image_pub.publish(image_out)
		        mov_fwd.publish(count) #publishes data on topic 'go_ahead' to indicate the dc motor to move forward 
                count=count+1 
        else:
            print("press 9 to capture image")
            cv2.waitKey(10)
            cv_image=bridge.imgmsg_to_cv2(data,"bgr8") #converts message of type image form ros images to opencv image
            cv2.imshow("",cv_image)
        

if __name__=='__main__':
    rospy.init_node('weed_detector_node')
    image_sub=rospy.Subscriber("image",Image,image_detection,queue_size=1)
    rospy.spin()
    cv2.destroyAllWindows()

