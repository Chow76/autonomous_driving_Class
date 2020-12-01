#!/usr/bin/env python
#coding=utf-8
import rospy
#倒入自定义的数据类型
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading

# GLOBAL VARIABLES
lane_vel = Twist()
speedcontrol = 1
angularScale = 6       # 180/30,该值越大，转弯的程度越大，该值越小，即倾向于维持直行
servodata=0
traffic_light_data=0

def thread_job():
    
    rospy.spin()

def lanecallback(msg):
    global lane_vel
    lane_vel = msg
    _servoCmdMsg = msg.angular.z * angularScale + 90
    global servodata
    servodata = min(max(0, _servoCmdMsg), 180)
    servodata=100-servodata*100/180
#rospy.loginfo('lane_vel.angular.z = %f',lane_vel.angular.z)

def lightcallback(msg):
    global traffic_light_data
    traffic_light_data = msg.data
#rospy.loginfo(rospy.get_caller_id() + "traffic_light_data is %s", traffic_light_data)

def lidarcallback(msg):
    global speedcontrol
    if msg.data:
       speedcontrol = 0
    else:
       speedcontrol = 1

def kinematicCtrl():
    
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    
    pub1 = rospy.Publisher('/bluetooth/received/manul', Int32 , queue_size=10)
    pub2 = rospy.Publisher('/auto_driver/send/direction', Int32 , queue_size=10)
    pub3 = rospy.Publisher('/auto_driver/send/speed', Int32 , queue_size=10)
    pub4 = rospy.Publisher('/auto_driver/send/gear', Int32 , queue_size=10)
    
    manul=0       # 0 - Automatic(自动); 1 - Manual (手动操控)
    speed=7      # SPEED (0～100之间的值)
    direction=50  # 0-LEFT-50-RIGHT-100 (0-49:左转，50:直行，51～100:右转)
    gear=1        # 1 - DRIVE, 2 - NEUTRAL, 3 - PARK, 4 - REVERSE
                  # 1:前进挡 2:空挡 3:停车挡 4:倒挡
    
    cmd_vel = Twist()
    flag=0
    p_flag=1
    servodata_list=[]
    n_loop=1
    
    rospy.init_node('kinematicCtrl', anonymous=True)
    
    add_thread = threading.Thread(target = thread_job)
    
    add_thread.start()
    
    rate = rospy.Rate(8) # 8Hz
    rospy.Subscriber("/lane_vel", Twist, lanecallback)  #节点名，数据类型，回调函数
    #rospy.Subscriber("/traffic_light", Int32, lightcallback)
    rospy.Subscriber("/has_obs", Bool, lidarcallback)
    
    #更新频率是1hz
    rospy.loginfo(rospy.is_shutdown())
    n = 1
    servodata_list = n * [servodata]
    while not rospy.is_shutdown():
        # KINEMATIC CONTROL CODE HERE
        servodata_list[0:n-1] = servodata_list[1:n]
        servodata_list[n-1] = servodata
        #servodata_mean = np.mean(servodata_list)*n
        servoSum = 0
        direction = servodata
        speed = 20*speedcontrol
        for i in servodata_list:
            servoSum += i
    
        servodata_mean = servoSum / n

       
        
        # 此处以“红灯停、绿灯行”为例，写下逻辑。
        # USE (traffic_light_data)
        # TO CHANGE: GEAR, DIRECTION AND SPEED.
    
        
        #if traffic_light_data == 0:
            #speed = 0
            #gear  = 2
        #if traffic_light_data == 1:
            #direction = servodata_mean
            #speed = 20
            #gear  = 1
            
        pub1.publish(manul)
        pub2.publish(direction)
        pub3.publish(speed)
        pub4.publish(gear)
        rate.sleep()

if __name__ == '__main__':
    kinematicCtrl()

