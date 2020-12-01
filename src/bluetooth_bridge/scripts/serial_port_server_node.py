#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
 
import rospy
import sys
import signal
import std_msgs.msg
import serial
import time 
import threading 
from std_msgs.msg import Int32
from std_msgs.msg import String
import struct


# topics
topic_from_bluetooth="/bluetooth/received/data"
#topic_from_auto_driver="/auto_driver/received/data"
topic_from_auto_driver_direction="/auto_driver/send/direction"
topic_from_auto_driver_speed="/auto_driver/send/speed"
topic_from_auto_driver_gear="/auto_driver/send/gear"
topic_from_auto_driver_beef="/auto_driver/send/beef"

topic_ActualMotorSpeed="/vcu/ActualMotorSpeed"
topic_ActualVehicleMode="/vcu/ActualVehicleMode"
topic_ActualVehicleDirection="/vcu/ActualVehicleDirection"
topic_SupersonicDistance="/vcu/SupersonicDistance"
topic_aX="/vcu/aX"
topic_aY="/vcu/aY"
topic_aZ="/vcu/aZ"
topic_alphaX="/vcu/alphaX"
topic_alphaY="/vcu/alphaY"
topic_alphaZ="/vcu/alphaZ"
topic_BX="/vcu/BX"
topic_BY="/vcu/BY"
topic_BZ="/vcu/BZ"
topic_thetaX="/vcu/thetaX"
topic_thetaY="/vcu/thetaY"
topic_thetaZ="/vcu/thetaZ"
topic_batteryVoltage = "/vcu/batteryVoltage"
topic_motorTemperature = "/vcu/motorTemperature"

topic_from_vcu="/bluetooth/send"
#topic_direction = "/bluetooth/received/direction"        # Received data from Bluetooth will
#topic_speed = "/bluetooth/received/speed"        # be published to this topic.
#topic_gear = "/bluetooth/received/gear"
#topic_manual = "/bluetooth/received/manual"
#topic_beep = "/bluetooth/received/beep"



def sigint_handler(signal, frame):
    print(" ")
    print("Interrupt!\n")
    print("Terminated")
    if ser.isOpen():
        ser.close()
#    ser.close()
    sys.exit(0)

#serial init
node_name='serial_port' 
serialPort = "/dev/ttyUSB0"
baudRate = 1000000
ser = serial.Serial(serialPort, baudRate)
#ser = serial.Serial(serialPort, baudRate, timeout=0.4)
print("serial port is %s ,baudRate is %d" % (serialPort, baudRate))
time.sleep(1)
signal.signal(signal.SIGINT, sigint_handler)
#default          head,direction,speed,gear,manul,beef,crc,bit
#auto_driver_data=[0xaa,50       ,0    ,3   ,0    ,0   ,0  , 0]
auto_driver_data=chr(0xaa)+chr(50)+chr(0)+chr(3)+chr(0)+chr(0)+chr(0)+chr(0)

flag_manul=0
 
#callback relate to subscriber 
def callback_bluetooth(message):
    global flag_manul
    print("bluetooth_data:",message.data)
    flag_manul=ord(message.data[4])
    if flag_manul==1:
        print "Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(message.data[0]),ord(message.data[1]),ord(message.data[2]),ord(message.data[3]),ord(message.data[4]),ord(message.data[5]),ord(message.data[6]))
        ser.write(message.data[0:8])
        ser.flush()
def callback_direction(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_Direction:",message.data,chr(message.data))
   # auto_driver_data[1]=chr(message.data)
    auto_driver_data =auto_driver_data[0]+chr(message.data)+auto_driver_data[2:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()

def callback_speed(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_speed:",message)
#    auto_driver_data[2]=str(message.data)
    auto_driver_data =auto_driver_data[0:2]+chr(message.data)+auto_driver_data[3:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Receiived from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()
def callback_gear(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_gear:",message.data)
 #   auto_driver_data[3]=str(message.data)
    auto_driver_data =auto_driver_data[0:3]+chr(message.data)+auto_driver_data[4:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()
def callback_beep(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_beep:",message.data)
  #  auto_driver_data[5]=str(message.data)
    auto_driver_data =auto_driver_data[0:5]+chr(message.data)+auto_driver_data[6:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()

def thread_job():
    rospy.spin()
 
 
def listener():
    #node init
    rospy.init_node(node_name, anonymous=True)  
    print("init_node:",node_name)
    #publish topic
    vcu_data_pub = rospy.Publisher(topic_from_vcu, String, queue_size = 10)
    vcu_ActualMotorSpeed_pub = rospy.Publisher(topic_ActualMotorSpeed, String, queue_size = 10)
    vcu_ActualVehicleMode_pub=rospy.Publisher(topic_ActualVehicleMode, Int32, queue_size = 10)
    vcu_ActualVehicleDirection_pub=rospy.Publisher(topic_ActualVehicleDirection, Int32, queue_size = 10)
    vcu_SupersonicDistance_pub=rospy.Publisher(topic_SupersonicDistance, Int32, queue_size = 10)
    vcu_aX_pub=rospy.Publisher(topic_aX, Int32, queue_size = 10)
    vcu_aY_pub=rospy.Publisher(topic_aY, Int32, queue_size = 10)
    vcu_aZ_pub=rospy.Publisher(topic_aZ, Int32, queue_size = 10)
    vcu_alphaX_pub=rospy.Publisher(topic_alphaX, Int32, queue_size = 10)
    vcu_alphaY_pub=rospy.Publisher(topic_alphaY, Int32, queue_size = 10)
    vcu_alphaZ_pub=rospy.Publisher(topic_alphaZ, Int32, queue_size = 10)
    vcu_BX_pub=rospy.Publisher(topic_BX, Int32, queue_size = 10)
    vcu_BY_pub=rospy.Publisher(topic_BY, Int32, queue_size = 10)
    vcu_BZ_pub=rospy.Publisher(topic_BZ, Int32, queue_size = 10)
    vcu_thetaX_pub=rospy.Publisher(topic_thetaX, Int32, queue_size = 10)
    vcu_thetaY_pub=rospy.Publisher(topic_thetaY, Int32, queue_size = 10)
    vcu_thetaZ_pub=rospy.Publisher(topic_thetaZ, Int32, queue_size = 10)
    vcu_batteryVoltage_pub=rospy.Publisher(topic_batteryVoltage, Int32, queue_size = 10)
    vcu_motorTemperature_pub=rospy.Publisher(topic_motorTemperature, Int32, queue_size = 10)
    #subscriber topic
    rospy.Subscriber(topic_from_bluetooth, String, callback_bluetooth)
    #rospy.Subscriber(topic_from_auto_driver, String, callback_auto_driver)
    rospy.Subscriber(topic_from_auto_driver_direction, Int32, callback_direction) 
    rospy.Subscriber(topic_from_auto_driver_speed, Int32, callback_speed) 
    rospy.Subscriber(topic_from_auto_driver_gear, Int32, callback_gear) 
    #rospy.Subscriber(topic_manual, Int32, callback_manual) 
    rospy.Subscriber(topic_from_auto_driver_beef, Int32, callback_beep) 
    #
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    rate = rospy.Rate(1000)
    n=0
    count=0
    global ser
    while not rospy.is_shutdown():
        try:
            #ser = serial.Serial(serialPort, baudRate, timeout=0.4)
            if count!=0:
                #ser = serial.Serial(serialPort, baudRate, timeout=0.4)
                ser = serial.Serial(serialPort, baudRate)
                print("restart serial")

                #ser.open()
                ser.timeout==2
#            ser.baudrate=9600
            count+=1
            while not rospy.is_shutdown():
                get_str = ser.read(34)
                n+=1
                ser.flushInput()
                s=struct.Struct('<34b')
                s3=struct.Struct('<h')
                s1=struct.Struct('<2b13h')
                s2=struct.Struct('<b')
                unpack_data=s.unpack(get_str)
                unpack_data3=s3.unpack(get_str[1:3])
                unpack_data1=s1.unpack(get_str[3:31])
                data_Android=str(unpack_data3[0])+","+str(unpack_data1[0])
                print("head==",unpack_data[0])
                print(get_str)

                if True :
                   #print("vcu data publisher")
                   #if n== 2:
                   #    vcu_data_pub.publish(data_Android)
                   vcu_data_pub.publish(get_str[0:])
                   print("vcu data publisher")
                    #   n=0
                   vcu_ActualMotorSpeed_pub.publish(unpack_data3[0])
                   vcu_ActualVehicleMode_pub.publish(unpack_data1[0])
                   vcu_ActualVehicleDirection_pub.publish(unpack_data1[1])
                   vcu_SupersonicDistance_pub.publish(unpack_data1[2])
                   vcu_aX_pub.publish(unpack_data1[3])
                   vcu_aY_pub.publish(unpack_data1[4])
                   vcu_aZ_pub.publish(unpack_data1[5])
                   vcu_alphaX_pub.publish(unpack_data1[6])
                   vcu_alphaY_pub.publish(unpack_data1[7])
                   vcu_alphaZ_pub.publish(unpack_data1[8])
                   vcu_BX_pub.publish(unpack_data1[9])
                   vcu_BY_pub.publish(unpack_data1[10])
                   vcu_BZ_pub.publish(unpack_data1[11])
                   vcu_thetaX_pub.publish(unpack_data1[12])
                   vcu_thetaY_pub.publish(unpack_data1[13])
                   vcu_thetaZ_pub.publish(unpack_data1[14])
                   vcu_batteryVoltage_pub.publish(unpack_data[31])
                   vcu_motorTemperature_pub.publish(unpack_data[32])
                   print(unpack_data[0],unpack_data3[0],unpack_data1[0],unpack_data1[1],unpack_data1[2],unpack_data1[3],unpack_data1[4],unpack_data1[5],unpack_data1[6],unpack_data1[7],unpack_data1[8],unpack_data1[9],unpack_data1[10],unpack_data1[11],unpack_data1[12],unpack_data1[13],unpack_data1[14],unpack_data[31],unpack_data[32])
                #rate.sleep()

        except Exception, e:
            print("have a serial error")
            if ser.isOpen():
                ser.close()

if __name__ == '__main__':
    listener()
 
 
########################
 
