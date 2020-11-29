#!/usr/bin/env python
# license removed for brevity
import rospy
import socket
from std_msgs.msg import Int32

       
def talker():
    host='192.168.2.111'
    port=7777
    
    msg_traffic_light=0
    
    pub1 = rospy.Publisher('/traffic_light', Int32 , queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
        s.connect((host,port))      
        data=s.recv(1024)    
        print(data)    
        
        #socket数据判断, PLS FINISH
        if data == "red_stop":
            msg_traffic_light = 0
        if data == "green_go":
            msg_traffic_light = 1

        pub1.publish(msg_traffic_light)
        s.close()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
