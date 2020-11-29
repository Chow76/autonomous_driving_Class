#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


#距离映射
x_cmPerPixel = 90/665.00
y_cmPerPixel = 81/680.00
roadWidth = 665

y_offset = 50.0 #cm

#轴间距
I = 58.0
#摄像头坐标系与车中心间距
D = 18.0
#计算cmdSteer的系数
k = -19

fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
videoNameString = time.strftime("%Y%m%d-%H%M%S_video.avi", time.localtime())
class camera:
    def __init__(self):

        self.camMat = []
        self.camDistortion = []

        self.cap = cv2.VideoCapture('/dev/video10')
        # self.cap = cv2.VideoCapture('video2.avi')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.vout = cv2.VideoWriter()
        self.vout.open(videoNameString, fourcc, 10, (1280, 720), True)

        self.imagePub = rospy.Publisher('images', Image, queue_size=1)
        self.cmdPub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        self.cam_cmd = Twist()
        self.cvb = CvBridge()
        
        src_points = np.array([[3,570], [387,460], [906,452], [1041,485]], dtype="float32")
        dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")
        self.M = cv2.getPerspectiveTransform(src_points, dst_points)

        self.aP = [0.0, 0.0]
        self.lastP = [0.0, 0.0]
        self.Timer = 0
    
    def __del__(self):
        self.cap.release()
        self.vout.release()

    def spin(self):
        ret, img = self.cap.read()
        if ret == True:
	    print('1')
            self.vout.write(img)
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3,3), np.uint8)
            gray_img = cv2.erode(gray_img, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_img)
            origin_thr[(gray_img >= 125)] = 255
            
            binary_warped = cv2.warpPerspective(origin_thr, self.M, (1280, 720), cv2.INTER_LINEAR)
            cv2.imshow('binary_warped',binary_warped)
            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] /2):, :], axis=0)
            lane_base = np.argmax(histogram_x)
            midpoint_x = int(histogram_x.shape[0]/2)

            histogram_y = np.sum(binary_warped[0:binary_warped.shape[0], :], axis=1)
            midpoint_y = 320 #int(histogram.shape[0]/2)
            upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
            lower_half_histSum = np.sum(histogram_y[midpoint_y: ])
            try:
                hist_sum_y_ratio = (upper_half_histSum)/(lower_half_histSum)
            except:
                hist_sum_y_ratio = 1
            #print(hist_sum_y_ratio)


            nwindows = 10
            window_height = int(binary_warped.shape[0] / nwindows)
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            lane_current = lane_base
            margin = 100
            minpix = 25

            lane_inds = []

            for window in range(nwindows):
                win_y_low = binary_warped.shape[0] - (window + 1) * window_height
                win_y_high = binary_warped.shape[0] - window * window_height
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                            nonzerox < win_x_high)).nonzero()[0]

                lane_inds.append(good_inds)
                if len(good_inds) > minpix:
                    lane_current = int(np.mean(nonzerox[good_inds]))  ####
                elif window>=3:
                    break

            lane_inds = np.concatenate(lane_inds)

            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

            # calculate the aimPoint
            if (pixelX.size == 0):
                return

            a2, a1, a0 = np.polyfit(pixelY,pixelX, 2)

            aveX = np.average(pixelX)
            
            frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
            aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

            # 计算aimLaneP处斜率，从而得到目标点的像素坐标
            lanePk = 2 * a2 * aimLaneP[0] + a1
            if(abs(lanePk)<0.1):
                if lane_base >= midpoint_x:
                    LorR = -1
                else:
                    if hist_sum_y_ratio < 0.1:
                        LorR = -0.5
                    else:
                        LorR = 2.4
                self.aP[0] = aimLaneP[0] +LorR  * roadWidth / 2
                self.aP[1] = aimLaneP[1]
            else:
                if (2 * a2 * aveX + a1) > 0:  # 斜率大于0


                    if a2 > 0:
                            # x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)  # 求截距

                    else:
                        x_intertcept= (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


                else:  # 斜率小于0
                    if a2 > 0:
                            # x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                    else:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


                if (x_intertcept > 599):    #求截距，即车道线端点
                    LorR = -0.3# RightLane
                else:
                    LorR = 0.6 # LeftLane

                k_ver = - 1 / lanePk
                theta = math.atan(k_ver)
                self.aP[0] = aimLaneP[0] + math.cos(theta) * (LorR) * roadWidth / 2
                self.aP[1] = aimLaneP[1] + math.sin(theta) * (LorR) * roadWidth / 2

            self.aP[0] = (self.aP[0] - 599) * x_cmPerPixel
            self.aP[1] = (680 - self.aP[1]) * y_cmPerPixel + y_offset

            # 计算目标点的真实坐标
            if (self.lastP[0] > 0.001 and self.lastP[1] > 0.001):
                if (((self.aP[0] - self.lastP[0]) ** 2 + (
                        self.aP[1] - self.lastP[1]) ** 2 > 2500) and self.Timer < 2):  # To avoid the mislead by walkers
                    self.aP = self.lastP[:]
                    self.Timer += 1
                else:
                    self.Timer = 0

            self.lastP = self.aP[:]
            steerAngle = math.atan(2 * I * self.aP[0] / (self.aP[0] * self.aP[0] + (self.aP[1] + D) * (self.aP[1] + D)))

            self.cam_cmd.angular.z = k * steerAngle
            #print("steerAngle=", steerAngle)
            print(LorR)            
            self.cmdPub.publish(self.cam_cmd)
            self.imagePub.publish(self.cvb.cv2_to_imgmsg(binary_warped))  # binary_warped
            binary_warped2 = cv2.resize(binary_warped,(640, 360))
            cv2.imshow('binary_warped', binary_warped2)
            cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(10)
    
    try:
        cam = camera()
        print(rospy.is_shutdown())  # FALSE
        while not rospy.is_shutdown():
            cam.spin()
            #print('betweeen == cam.spin ==')
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass


