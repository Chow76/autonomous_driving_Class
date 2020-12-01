#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
 
import rospy
import std_msgs.msg
from std_msgs.msg import Int32
from std_msgs.msg import String
from playsound import playsound


# topics
topic_soundRequest="/soundRequest"
node_name="sound_server_node"

def callback_sound(message):
    print("enter callback_sound/n")
    if message.data==0:
        pass
        print("pass/n")
    elif message.data==1:
        playsound("/home/pi/Music/sound/didi.mp3")
        print("didi/n")
    elif message.data==2:
        playsound("/home/pi/Music/sound/kj.mp3")
        print("kaijichengong")
    elif message.data==3:
        playsound("/home/pi/Music/sound/gj.mp3")



 
def listener():
    #node init
    rospy.init_node(node_name, anonymous=True)  
    print("init_node:",node_name)
    #subscriber topic
    rospy.Subscriber(topic_soundRequest, Int32, callback_sound) 
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
        
 
 
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("enter except,Subscriber")
 
 
 
########################
 
