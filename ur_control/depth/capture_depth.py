#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
#from ImageProcessor import ImageProcessor
from sklearn import preprocessing
import rospkg
import pandas
import math

class RosImgRecord:
    def __init__(self):
        # initialize OpenCV Bridge
        self.bridge = CvBridge()
        

	   # set subscriber
        self.colorimageSubscriber = rospy.Subscriber("/camera/depth/image_raw",Image, self.RecordcolorImage, queue_size= 20)
    
        self.keySubscriber = rospy.Subscriber("/keys", String, self.key_recv, queue_size = 1)
	   # set publisher
	   #self.imagePublisher = rospy.Publisher("/processed/color/image_raw",Image, queue_size= 20)
        self.imgCount = 0
        self.color_key = 0
        
    def key_recv(self, data):
        self.color_key = 1
        self.ir_key = 1
        self.imgCount += 1

         
    def RecordcolorImage(self, data):     
        try:
            Img = self.bridge.imgmsg_to_cv2(data, "16UC1")
            #cv2.imshow('img', Img)
            if self.color_key == 1:

		Img=Img.squeeze()
		print Img.shape
		#保存成图片格式
		cv2.imwrite(r"{}.bmp".format(self.imgCount), Img)
		data2=pandas.DataFrame(Img)
		data2.to_csv(r"{}_origin.csv".format(self.imgCount))
		#归一化
		Img_scale=Img[Img>0]
		avrg=math.floor(np.mean(Img_scale))
		var=np.std(Img_scale)
		print avrg,var
		index=Img==0
		Img[index]=avrg
		Img_scale=(Img-avrg)/var
		#index1=abs(Img_scale)<0.05
		#Img_scale[index1]=0
		#保存成csv格式
		data1=pandas.DataFrame(Img_scale)
		data1.to_csv(r"{}.csv".format(self.imgCount))
		#保存成txt格式
                #np.savetxt(r"{}.txt".format(self.imgCount), Img)

                self.color_key = 0
            cv2.waitKey(5)
        except CvBridgeError as e:
            print e
             
             
     
             


def main(args=None):
	rospy.init_node('processed', anonymous=True)
	Node = RosImgRecord()	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
