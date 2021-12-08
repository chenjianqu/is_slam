# _*_ coding: UTF-8 _*_
import socket
import sys
if('/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path):
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy
from my_eval import yolact
import tqdm
import os
import time


if __name__ == '__main__':
    timeSum=0.0
    counter=0

    pose_path="/media/chen/chen/GraduationProject/data/track/freiburg1_xyz/map_keyframe.txt"
    with open(pose_path,'r') as f:
        poseStr=f.read()

    data_path="/media/chen/chen/SLAM/datasets/rgbd_dataset_freiburg1_xyz/rgb"
    result_path="/media/chen/chen/GraduationProject/data/instance_segmentation/freiburg1_xyz"
    for name in tqdm.tqdm(os.listdir(data_path)):
        timeStr=name[:-4]
        if(timeStr not in poseStr):
            continue
        counter+=1
        print(counter,name)

        img_path=os.path.join(data_path,name)
        img=cv2.imread(img_path,1)
        start=time.time()
        result=yolact(img)
        timeSum+=(time.time()-start)
        save_path=os.path.join(result_path,name)
        cv2.imwrite(save_path,result)

    print(timeSum/counter)



