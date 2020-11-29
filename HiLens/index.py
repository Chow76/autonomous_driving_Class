#! /usr/bin/python3.7

import cv2
import hilens

import os
import sys
import time
from utils import *
from socket import *

HOST = ''
PORT = 7777
bufsize = 1024
socket_3399 = socket(AF_INET,SOCK_STREAM)
socket_3399.bind((HOST,PORT))
socket_3399.listen()


def run():
    # 系统初始化，参数要与创建技能时填写的检验值保持一致
    hilens.init("hello")
    

    # 初始化摄像头
    camera  = hilens.VideoCapture()
    display = hilens.Display(hilens.HDMI)
    
    # 初始化模型
    model_path = hilens.get_model_dir() + "convert-light.om"
    model      = hilens.Model(model_path)
    
    while True:
        ##### 1. 设备接入 #####
        input_yuv = camera.read() # 读取一帧图片(YUV NV21格式)
        
        ##### 2. 数据预处理 #####
        img_rgb = cv2.cvtColor(input_yuv, cv2.COLOR_YUV2RGB_NV21) # 转为RGB格式
        img_preprocess, img_w, img_h = preprocess(img_rgb) # 缩放为模型输入尺寸
    
        ##### 3. 模型推理 #####
        output = model.infer([img_preprocess.flatten()])
        
        ##### 4. 结果输出 #####
        bboxes  = get_result(output, img_w, img_h) # 获取检测结果
        img_rgb, labelName = draw_boxes(img_rgb, bboxes)      # 

        socketSendMsg(socket_3399,labelName)
        output_yuv = hilens.cvt_color(img_rgb, hilens.RGB2YUV_NV21)
        display.show(output_yuv) # 显示到屏幕上
    

    socket_3399.close()
    hilens.terminate()


if __name__ == "__main__":
    run()
    
