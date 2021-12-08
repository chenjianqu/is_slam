# _*_ coding: UTF-8 _*_
import socket
import sys
if('/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path):
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy
from my_eval import yolact
from time import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("",12345))
s.listen(5)

def recvall(sock, count):
	buf = b''
	while count:
		newbuf = sock.recv(count)
		if not newbuf: return None
		buf += newbuf
		count -= len(newbuf)
	return buf

def recvimage(conn,addr):
	length = recvall(conn,16)
	if(length is None):
		return
	stringData = recvall(conn, int(length))
	data = numpy.fromstring(stringData, dtype='uint8')
	decimg=cv2.imdecode(data,1)
	return decimg

def sendimage(conn,img):
	result, imgencode = cv2.imencode('.png',img, [int(cv2.IMWRITE_PNG_COMPRESSION),0]) #采用最高质量的png压缩，防止失真
	data = numpy.array(imgencode)
	stringData = data.tostring()
	conn.send( str(len(stringData)).ljust(16).encode());
	conn.send( stringData );





count=0

try:
	while True:
		print("等待连接...")
		conn, addr = s.accept()
		print("图像流传输开始")
		while 1:
			img=recvimage(conn, addr)
			if(img is None):
				print("图像流传输结束")
				break
			t_start=time()
			img=yolact(img)
			print(time()-t_start)
			sendimage(conn,img)
			print(count)
			count+=1
except KeyboardInterrupt:#捕获ctrl c
	pass
s.close()
print("程序结束")