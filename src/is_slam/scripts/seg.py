#!/usr/bin/env python
# -*- coding: utf-8 -*-

#########################################################
# Copyright (C) 2022, Chen Jianqu, Shanghai University
#
# This file is part of is_slam.
#
# Licensed under the MIT License;
# you may not use this file except in compliance with the License.
#########################################################

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from segmentation import Segmentation
import message_filters



class image_converter:
	def __init__(self):    
		self.seg=Segmentation()
		rospy.loginfo("Socket传输线程启动完成")
		# 创建cv_bridge，声明图像的发布者和订阅者
		self.image_pub = rospy.Publisher("/ps/rgb", Image, queue_size=1)
		self.depth_pub = rospy.Publisher("/ps/depth", Image, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = message_filters.Subscriber("/orbslam2/rgb", Image)
		self.depth_sub = message_filters.Subscriber("/orbslam2/depth", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.1, allow_headerless=True)
		self.ts.registerCallback(self.callback)
		self.counter=0

	def callback(self,image_msg,depth_msg):
		# 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
		except CvBridgeError as e:
			print e
		self.counter+=1
		#self.seg.push_msg((cv_image,image_msg.header,depth_msg)) #写入等待队列
		#rospy.loginfo("当前等待队列长度："+str(self.seg.get_waiting_len()))

	def send_result(self,img,img_header,depth_msg):
		# 再将opencv格式额数据转换成ros image格式的数据发布
		try:
			img_msg=self.bridge.cv2_to_imgmsg(img, "bgr8")
			img_msg.header=img_header
			self.image_pub.publish(img_msg)
			self.depth_pub.publish(depth_msg)
		except CvBridgeError as e:
			print e

if __name__ == '__main__':
	rospy.init_node("ps_node")
	ct=image_converter()
	rospy.loginfo("PS节点——初始化完成")
	#视频写入
	#fourcc = cv2.VideoWriter_fourcc('X', 'V','I','D')
	#output_movie = cv2.VideoWriter('test.avi', fourcc, 5, (640, 480))
	rate=rospy.Rate(10) #10Hz
	while not rospy.is_shutdown():
		if(ct.seg.get_result_len()>0): #查询是否有分割结果
			rospy.loginfo("PS节点——等待队列长度："+str(ct.seg.get_waiting_len())+
				" 结果队列长度："+str(ct.seg.get_result_len()))
			(img,img_header,depth_msg)=ct.seg.pop_result()
			ct.send_result(img,img_header,depth_msg)
		rate.sleep()
	rospy.loginfo("正在关闭Socket传输线程")
	ct.seg.set_state(False) #关闭tcp传输线程
