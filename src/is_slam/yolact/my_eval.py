from data import COCODetection, get_label_map, MEANS, COLORS
from yolact import Yolact
from utils.augmentations import BaseTransform, FastBaseTransform, Resize
from utils.functions import MovingAverage, ProgressBar
from layers.box_utils import jaccard, center_size, mask_iou
from utils import timer
from utils.functions import SavePath
from layers.output_utils import postprocess, undo_image_transformation
import pycocotools

from data import cfg, set_cfg, set_dataset

import numpy as np
import torch
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
import argparse
import time
import random
import cProfile
import pickle
import json
import os
from collections import defaultdict
from pathlib import Path
from collections import OrderedDict
from PIL import Image

import tqdm

import matplotlib.pyplot as plt
import sys
if('/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path):
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


def prep_display(dets_out, img, h, w, undo_transform=True, class_color=False, mask_alpha=1.0, fps_str=''):
    if undo_transform:
        img_numpy = undo_image_transformation(img, w, h)
        img_gpu = torch.Tensor(img_numpy).cuda()
    else:
        img_gpu = img / 255.0
        h, w, _ = img.shape

    img_gpu=img_gpu.sum(dim=2)/3.0 #将各通道像素相加再除以3,得到单通道图像 
    img_gpu=img_gpu.view(h,w,1)
    #print("img_gpu"+str(img_gpu.size()))

    on_gpu=img_gpu.device.index
    
    save = cfg.rescore_bbox
    cfg.rescore_bbox = True
    #得到四个张量：
    # classes [100]: The class idx for each detection.
    # scores  [100]: The confidence score for each detection.
    # boxes   [100, 4]: The bounding box for each detection in absolute point form.
    # masks   [100, h, w]: Full image masks for each detection.
    t = postprocess(dets_out, w, h, visualize_lincomb = False,crop_masks= True,score_threshold   = 0)
    cfg.rescore_bbox = save

    #得到大于0.5置信度的实例
    effectiveDetect=torch.ge(t[1],0.5)#得到大于0.5置信度的蒙板
    detect_num=effectiveDetect.sum()#得到大于0.5置信度的检测的数量
    idx = t[1].argsort(0, descending=True)[:detect_num] #该图片中取置信度最高的前detect_num个实例
    masks = t[3][idx] #取出置信度最高的detect_num个实例的mask 
    classes, scores, boxes = [x[idx].cpu().numpy() for x in t[:3]] #将这detect_num个实例的类别、置信度、包围框 取出放到numpy数组中

    n = classes.shape[0]
    for j in range(n):
        if scores[j] < 0:
            n = j
            break
    #存在实例时
    if n > 0:
        masks = masks[:n, :, :, None]#[n, h, w, 1],实例区域是1,其余区域是0
        
        class_colors_=[torch.Tensor((254-classes[j],254-j)).to(on_gpu).float().view(1, 1, 1, 2)/255.0 for j in range(n)] #得到各实例的类别,类别0对应的是254像素值
        class_colors = torch.cat(class_colors_, dim=0) #colors的维度[n,1,1,2]
        class_mask = masks.repeat(1,1,1,2)* class_colors  #masks_color的维度[n, h, w, 2],实例区域外的像素值为0
        class_mask=class_mask.sum(dim=0) #将各个实例mask合成为一个mask
        img_gpu=torch.cat((img_gpu,class_mask),dim=2)#合成一个3通道的图片
    else:
        class_mask=torch.zeros((h, w,2)).to(on_gpu).float()
        img_gpu=torch.cat((img_gpu,class_mask),dim=2)

    img_numpy = (img_gpu * 255).byte().cpu().numpy()
    return img_numpy



print("正在加载模型")
trained_model='weights/yolact_resnet50_54_800000.pth'
model_path = SavePath.from_str(trained_model)
config = model_path.model_name + '_config'
set_cfg(config)

with torch.no_grad():
    cudnn.fastest = True
    torch.set_default_tensor_type('torch.cuda.FloatTensor')
    dataset = None        
    print('Loading model...', end='')
    net = Yolact()
    net.load_weights(trained_model)
    net.eval()
    print(' Done.')

    net = net.cuda()
    net.detect.use_fast_nms = True
    net.detect.use_cross_class_nms = False
    cfg.mask_proto_debug = False

print("模型初始化完成")

def yolact(img):
    with torch.no_grad():
        #读取图片
        frame = torch.from_numpy(img).cuda().float()
        #图像预处理
        batch = FastBaseTransform()(frame.unsqueeze(0)) 
        #预测
        preds = net(batch)
        #提取结果，并绘制到图片上
        img_numpy = prep_display(preds, frame, None, None, undo_transform=False)
        return img_numpy
        #矩阵维度转换
        #img_numpy = img_numpy[:, :, (2, 1, 0)]



if __name__ == '__main__':
    tmp_str='''
    img_path="1305031537.539497.png"
    img=cv2.imread(img_path,1)
    result=yolact(img)
    cv2.imwrite("test.png",result)
'''

    data_path="/media/chen/chen/SLAM/datasets/rgbd_dataset_freiburg1_xyz/rgb"
    #result_path="/media/chen/chen/SLAM/datasets/rgbd_dataset_freiburg2_pioneer_360/instance"
    for name in tqdm.tqdm(os.listdir(data_path)):
        img_path=os.path.join(data_path,name)
        img=cv2.imread(img_path,1)
        result=yolact(img)
        save_path=os.path.join(result_path,name)
        cv2.imwrite(save_path,result)

