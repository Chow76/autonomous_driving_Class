#!/usr/bin/env python
# -*- coding: utf-8 -*-

## @package docstring
#  This package provides the bridge between Bluetooth and ROS, both ways.
#  Initially it receives "String" messages and sends "String" messages
#

import rospy
import math
import sys
import signal
import bluetooth
import select
import time
from std_msgs.msg import String
from std_msgs.msg import Int32
import serial
import struct
import threading
import struct 
#--------------------------------- Constants ----------------------------------#

TAG       = "Bluetooth Bridge Node:"              ## Node verbosity tag
node_name = "bluetooth_bridge"                    ## ROS node name
version_num="2020/8/10/09/30"

#------------------------------------------------------------------------------#

#serial_node_name='serial_port'

send_data=""

#serialPort = "/dev/ttyUSB0"
#baudRate = 1000000;
#ser = serial.Serial(serialPort, baudRate, timeout=0.5)
print("This software version is %s" % (version_num))


get_str="0123456789"

## Application class
#
class Application:
    ## "Is application running" flag
    is_running   = True
    ## "Is connection established" flag
    is_connected = False

    is_normal_sending = False

    ## Input topics
    input_topic = "/bluetooth/send"             # Send a string messsage to this
                                                # topic to send it via  Bluetooth.
    sound_topic = "/soundRequest"             # Send a string messsage to this

    ## Output topics
    output_topic="/bluetooth/received/data"
    output_topic_direction = "/bluetooth/received/direction"        # Received data from Bluetooth will
    output_topic_speed = "/bluetooth/received/speed"        # be published to this topic.
    output_topic_gear = "/bluetooth/received/gear"
    output_topic_manual = "/bluetooth/received/manual"
    output_topic_beep = "/bluetooth/received/beep"
    
    status_topic = "/bluetooth/status"
    direction=0
    count =0

    ## Bluetooth channel
    bt_channel = 22                             # IMPORTANT! Mae sure this is THE SAME
                                                # as was used diring
                                                # sdptool add --channel=<number> SP comand.
                                                # Also, use this command before launching
                                                # this node if you have rebooted your robot.

    ## Init function
    def __init__(self):
        # Assigning the SIGINT handler
        signal.signal(signal.SIGINT, self.sigint_handler)

        #print math.sin(10)

        # Starting the node
        rospy.init_node(node_name, anonymous=False)

        # Getting parameters
        #self.input_topic  = rospy.get_param("~send_topic", self.input_topic)
        
        #self.output_topic_direction = rospy.get_param("~recv_topic", self.output_topic_direction)
        #self.output_topic_speed = rospy.get#self.direction#self.direction_param("~recv_topic", self.output_topic_speed)
        #self.output_topic_gear = rospy.get_param("~recv_topic", self.output_topic_gear)
        #self.output_topic_manual = rospy.get_param("~recv_topic", self.output_topic_manual)
        #self.output_topic_beep = rospy.get_param("~recv_topic", self.output_topic_beep)
        
        #self.status_topic = rospy.get_param("~status_topic", self.status_topic)
        #self.bt_channel   = rospy.get_param("~rfcomm_channel", self.bt_channel)

        #print TAG, "param: input_topic  =", self.input_topic
        
        #print TAG, "param: output_topic_direction =", self.output_topic_direction
        
        #print TAG, "param: output_topic_speed =", self.output_topic_speed
        #print TAG, "param: output_topic_gear =", self.output_topic_gear
        #print TAG, "param: output_topic_manual =", self.output_topic_manual
        #print TAG, "param: output_topic_beep =", self.output_topic_beep
        #print TAG, "param: status_topic =", self.status_topic
        #print TAG, "param: bt_channel   =", self.bt_channel

        # Subscribers
        self.sub        = rospy.Subscriber(self.input_topic, String, self.send_callback)

        # Publishers
        self.pub        = rospy.Publisher(self.output_topic, String, queue_size = 10)
        self.sound_pub        = rospy.Publisher(self.sound_topic, Int32, queue_size = 10)
        self.direction_pub        = rospy.Publisher(self.output_topic_direction, Int32, queue_size = 10)
        self.speed_pub        = rospy.Publisher(self.output_topic_speed, Int32, queue_size = 10)
        self.gear_pub        = rospy.Publisher(self.output_topic_gear, Int32, queue_size = 10)
        self.manual_pub        = rospy.Publisher(self.output_topic_manual, Int32, queue_size = 10)
        self.beep_pub        = rospy.Publisher(self.output_topic_beep, Int32, queue_size = 10)
        
        
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size = 10)
        time.sleep(0.5)
        self.status_pub.publish("INIT")  
        rospy.Timer(rospy.Duration(0.5),self.timer_callback)
        self.sound_pub.publish(2)
        self.add_thread = threading.Thread(target = self.thread_job)
        self.add_thread.start()
        
        while self.is_running:
            try:
                # Starting the bluetooth server
                self.server_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
                # Listening for incoming connections
                self.server_sock.bind( ("", self.bt_channel) )
                #self.server_sock.bind( ("", self.bt_channel) )
                print TAG, "Waiting for incoming connections on port %d ..." % self.bt_channel
                self.status_pub.publish("LISTENING")
                self.server_sock.listen(1)
                # Accepting incoming connection
                self.client_sock, self.address = self.server_sock.accept()
                #print TAG, "Accepted connection from ", self.address
                self.status_pub.publish("CONNECTED: "+str(self.address))

                # [IMPORTANT] THIS IS HOW TO RECEIVE MESSAGE FROM BLUETOOTH
                # AND PUBLISH IT TO ROS

                # Running the loop to receive messages
                self.is_connected  = True
                while self.is_running:
                    ready = select.select([self.client_sock],[],[], 2)
                 #   print ready
                    if ready[0]:
                        data = self.client_sock.recv(1024)
                        self.is_normal_sending=True
                        #self.direction=math.cos(ord(data[1])//math.pi)*ord(data[6])
                    #    print TAG, "Received: ", data.encode('hex'),type(data.encode('hex'))
                        #print TAG, "Received: ", ord(data[0]),ord(data[1]),ord(data[2]),ord(data[3]),ord(data[4]),ord(data[5]),ord(data[6]))
                        print TAG, "Received: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(data[0]),ord(data[1]),ord(data[2]),ord(data[3]),ord(data[4]),ord(data[5]),ord(data[6]))
                        #if ((data[1]>>8)&1) :
                         #   print "is 1"
                        #else:
                         #    print "is 0"
                        if (ord(data[0])==0xaa)and(ord(data[7])==(ord(data[0])^ord(data[1])^ord(data[2])^ord(data[3])^ord(data[4])^ord(data[5])^ord(data[6]))):
                            self.pub.publish(data)
                            self.direction_pub.publish(ord(data[1]))    #/bluetooth/received/direction
                            #self.direction_pub.publish(self.direction)
                            self.speed_pub.publish(ord(data[2]))        #/bluetooth/received/speed
                            self.gear_pub.publish(ord(data[3]))         #/bluetooth/received/speed
                            self.manual_pub.publish(ord(data[4]))       #/bluetooth/received/manual
                            self.beep_pub.publish(ord(data[5]))         #/bluetooth/received/beep
                            if (ord(data[5])):
                                self.sound_pub.publish(1)

#                            self.client_sock.send(data[0:6])
                            global get_str
                            get_str=data

    #                        buffer=struct.pack("s",data)
                            #ser.write(data[0:8])
                            #ser.flush()
                        else:
                            print "CRC not pass"


                        self.client_sock.send(send_data)
                            
                        

            except Exception, e:
                self.is_connected = False
                self.server_sock.close()    #
                print TAG, "EXCEPTION:", str(e)
                self.status_pub.publish("EXCEPTION: "+str(e))
                #self.pub.publish(data)   #
                self.direction_pub.publish(0)    #/bluetooth/received/direction
                
                self.speed_pub.publish(0)        #/bluetooth/received/speed
                self.gear_pub.publish(0)         #/bluetooth/received/speed
                self.manual_pub.publish(0)       #/bluetooth/received/manual
                self.beep_pub.publish(0)         #/bluetooth/received/beep
                print TAG, "RESTARTING SERVER"
                time.sleep(0.1)


    ## SIGINT Signal handler, you need this one to interrupt your node
    def sigint_handler(self, signal, frame):
            print ""
            print TAG,"Interrupt!"
            self.status_pub.publish("SIGINT")
            self.is_running = False
            print TAG,"Terminated"
            sys.exit(0)    #
    def thread_job(self):
        rospy.spin()

    ## [IMPORTANT] THIS IS HOW TO SEND MESSAGES VIA BLUETOOTH
    ## Handler for the messages to be sent via bluetooth.
    def send_callback(self, message):
        if self.is_connected:
            print TAG, "Sending:", message.data
            s=struct.Struct('<34b')
            s3=struct.Struct('h')
            s1=struct.Struct('2b13h')
            unpack_data=s.unpack(message.data)
            unpack_data3=s3.unpack(message.data[1:3])
            unpack_data1=s1.unpack(message.data[3:31])
            #print("speed:",unpack_data3[0],"gear:",unpack_data1[0])
            #unpack_data=str(unpack_data3[0])+","+str(unpack_data1[0])
            print("head is",unpack_data[0])
            print("Vol:",unpack_data[31],"temp:",unpack_data[32])
            print(unpack_data3[0],unpack_data1[0])
            speed=unpack_data3[0]
            if speed<0:
                speed=-(speed)
            print(speed,unpack_data1[0])
            global get_str
            global send_data
           # get_str=get_str[0:7]+chr(speed/256)+chr(speed%256)+chr(unpack_data1[0])
            print("speed is %d,%d.gear is %d" %(unpack_data3[0]/256,unpack_data3[0]%256,unpack_data1[0]))
            #self.client_sock.send(get_str[0:10])
            send_data = message.data
            #self.client_sock.send(send_data)
            #self.client_sock.send(message.data)
            print(type(message.data))

    def timer_callback(self,event):
        if (self.is_connected==True)and(self.is_normal_sending==False):
            self.status_pub.publish("NODATA")
            self.direction_pub.publish(0)    #/bluetooth/received/direction
            self.speed_pub.publish(0)        #/bluetooth/received/speed
            self.gear_pub.publish(0)         #/bluetooth/received/gear
            self.manual_pub.publish(0)       #/bluetooth/received/manual
            self.beep_pub.publish(0)         #/bluetooth/received/beep
        else:
            self.status_pub.publish("SEND/RECEIVE")
        self.is_normal_sending=False
            
            
                                                      

#------------------------------------- Main -------------------------------------#

if __name__ == '__main__':
    print TAG,"Started"

    app = Application()

    print TAG,"Terminated"
