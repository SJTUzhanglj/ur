#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import cv2
import os
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
#from ImageProcessor import ImageProcessor
import rospkg
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math

class RosImgToOpenCVImageConvertor:
	def __init__(self):

		
		# initialize OpenCV Bridge
		self.bridge = CvBridge()

		# set subscriber
		self.imageSubscriber = rospy.Subscriber("/camera/rgb/image_color",Image, self.processMyImage, queue_size= 20)

  
  		# set subscriber
		self.depthimageSubscriber = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw",
                                          Image, 
                                          self.processDepthImage, 
                                          queue_size= 20)
		# set publisher
		#self.imagePublisher = rospy.Publisher("/processed/thresh_image",Image, queue_size= 20)
		self.objframePublisher = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
		self.imgCount = 0 
		self.depthimg = None
		self.dist_cam2obj = 0
		self.cam2obj_trans = geometry_msgs.msg.TransformStamped()
		self.cam2obj_trans.header.frame_id = "camera_frame"
		self.cam2obj_trans.child_frame_id = "object_frame"
		self.cam2obj_trans.transform.rotation.w = 1.0 
		pass


	def processDepthImage(self, data):
         try:
             self.depthimg = self.bridge.imgmsg_to_cv2(data, "16UC1")  
             self.depthimg = np.squeeze(self.depthimg, -1)
         except CvBridgeError as e:
             print e
     
	def processMyImage(self, data):
         def findtarget(thresh_img):
             #hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
             #thresh_img = cv2.Canny(thresh_img, 0, 255)
             kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(15, 15))   
             thresh = cv2.morphologyEx(thresh_img, cv2.MORPH_CLOSE, kernel)
             
             contours, hierarchy = cv2.findContours(thresh, 
                                                    cv2.RETR_EXTERNAL, 
                                                    cv2.CHAIN_APPROX_NONE)
             max_area = 0
             area_lim = 100000
             min_area = 20
             best_cnt = None
             for cnt in contours:
                 area = cv2.contourArea(cnt)
                 #if area < min_area:                    
                 if area > max_area and area > min_area and area < area_lim:
                     max_area = area
                     best_cnt = cnt
             if max_area == 0:
                 return None
            
             
             return cv2.boundingRect(best_cnt)

         
         try:
             opencvImg = self.bridge.imgmsg_to_cv2(data, "bgr8")             
             thresh = cv2.inRange(opencvImg, np.array((200,200,200)), np.array((255,255,255)))
             #cv2.imshow('th', thresh)
             ret = findtarget(thresh)
             targetmask = np.zeros(np.shape(opencvImg)[:2], np.uint8)
             if ret != None:
                 cv2.rectangle(targetmask, (ret[0], ret[1]), (ret[0]+ret[2], ret[1]+ret[3]), 255, -1)
             #cv2.imshow('target', targetmask)
			#print ('---------process done!-----------')
             #self.imagePublisher.publish(processedImg)
             try:
                 thresh = np.squeeze(thresh, -1)
             except:
                 pass
             mask = np.clip(targetmask, 0, 1) * np.clip(thresh, 0, 1)
             cv2.imshow('mask', mask*255)
             cv2.waitKey(2)
             
             if self.depthimg is not None:  
                 dense_depth = (self.depthimg* mask).flatten()
                 dense_depth = dense_depth[dense_depth > 0]
                 dense_depth = dense_depth[dense_depth < 2000]
                 if len(dense_depth) > 0:
                     self.dist_cam2obj = np.mean(dense_depth) 
                 #print self.dist_cam2obj
             #depth = np.asarray(self.depthimg, np.uint8) * mask
             #cv2.imshow('depth', depth) 
             #self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(opencvImg, "bgr8"))
             if not np.isnan(self.dist_cam2obj):
                 self.cam2obj_trans.header.stamp = rospy.Time.now()
                 self.cam2obj_trans.transform.translation.z = self.dist_cam2obj/1000.0
                 tfm = tf2_msgs.msg.TFMessage([self.cam2obj_trans])
                 self.objframePublisher.publish(tfm)
			# cv2.imwrite(self.curModulePath+"/examples/"+str(self.imgCount)+'.jpg', processedImg)
			# self.imgCount += 1
			#cv2.waitKey(5)
         except CvBridgeError as e:
             print e

def main(args=None):
	rospy.init_node('distance_estimate', anonymous=True)
	Node = RosImgToOpenCVImageConvertor()	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
