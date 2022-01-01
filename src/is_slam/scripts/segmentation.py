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


from threading import Thread,Lock
import socket
import cv2
import numpy
import time


def recvall(sock, count):
	buf = b''
	while count:
		newbuf = sock.recv(count)
		if not newbuf: return None
		buf += newbuf
		count -= len(newbuf)
	return buf

def recvimage(conn):
	length = recvall(conn,16)

	stringData = recvall(conn, int(length))
	data = numpy.fromstring(stringData, dtype='uint8')
	decimg=cv2.imdecode(data,1)
	return decimg

def sendimage(conn,img):
	result, imgencode = cv2.imencode('.png',img, [int(cv2.IMWRITE_PNG_COMPRESSION),0])#采用最高质量的png压缩，防止失真
	data = numpy.array(imgencode)
	stringData = data.tostring()
	conn.send( str(len(stringData)).ljust(16));
	conn.send( stringData );


class Segmentation:
	def __init__(self):
		self.msg_list=[]
		self.result_list=[]
		self.msg_list_lock=Lock()
		self.result_list_lock=Lock()

		self.is_run=True
		self.is_run_lock=Lock()

		#启动处理线程
		t = Thread(target=Segmentation.run, args=(self,))
		t.setDaemon(True) #设置守护线程：给每个子线程一个timeout的时间，让他去执行，时间一到，不管任务有没有完成，直接杀死。
		t.start()
		#t.join() #线程同步：主线程结束后，进入阻塞状态，等待其它线程结束

	def run(self):
		address = ('localhost', 12345)
		sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		sock.connect(address)
		counter=0
		while self.get_state():
			if(self.get_waiting_len()>0):
				img,img_header,depth_msg=self.pop_msg()
				sendimage(sock,img) #发送到服务端
				result=recvimage(sock) #接收结果
				#cv2.imwrite("/media/chen/chen/Robot/Data/InstanceSegmentation/test/"+str(counter)+".png",
				#	result,[int(cv2.IMWRITE_PNG_COMPRESSION),0])
				counter+=1
				self.push_result((result,img_header,depth_msg)) #放到结果队列
		sock.close()




	def get_state(self):
		state=True
		self.is_run_lock.acquire()
		state=self.is_run
		self.is_run_lock.release()
		return state

	def set_state(self,state):
		self.is_run_lock.acquire()
		self.is_run=state
		self.is_run_lock.release()


	def push_msg(self,msg):
		self.msg_list_lock.acquire()
		self.msg_list.append(msg)
		self.msg_list_lock.release()

	def pop_msg(self):
		msg=0
		self.msg_list_lock.acquire()
		msg=self.msg_list.pop(0)
		self.msg_list_lock.release()
		return msg

	def get_waiting_len(self):
		length=0
		self.msg_list_lock.acquire()
		length=len(self.msg_list)
		self.msg_list_lock.release()
		return length

	def push_result(self,msg):
		self.result_list_lock.acquire()
		self.result_list.append(msg)
		self.result_list_lock.release()

	def pop_result(self):
		msg=0
		self.result_list_lock.acquire()
		msg=self.result_list.pop(0)
		self.result_list_lock.release()
		return msg

	def get_result_len(self):
		length=0
		self.result_list_lock.acquire()
		length=len(self.result_list)
		self.result_list_lock.release()
		return length



