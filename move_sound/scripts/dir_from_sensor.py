#!/usr/bin/env python3

from cmath import sin
from tuning import Tuning
from std_msgs.msg import Float32, Int32MultiArray
import usb.core
import usb.util
import time
import rospy
import math
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
import geometry_msgs.msg
import tf
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np


devices = tuple(usb.core.find( find_all=True, idVendor=0x2886, idProduct=0x0018))
for i in range(len(devices)):
    print(devices[i].bus, devices[i].address)

if len(devices) > 1:
    dev1 = devices[0]
    dev2 = devices[1]
    Mic_tuning1 = Tuning(dev1)
    Mic_tuning2 = Tuning(dev2)
    dir = Int32MultiArray()
    rospy.init_node('doa_node')
    rate = rospy.Rate(20)
    pub = rospy.Publisher('sound_angle', Float32, queue_size=1)
    while not rospy.is_shutdown():

        vad1 = Mic_tuning1.is_voice()
        vad2 = Mic_tuning2.is_voice()
        if vad1 == 1 and vad2 == 1:
            dir1 = Mic_tuning1.direction
            dir1_orig = dir1
            if dir1 > 180:
                dir1 = dir1-360
            dir2 = Mic_tuning2.direction
            if dir2 > 180:
                dir2 = dir2-360
            dir2in1 = dir2 + 180  # sensor 2 angle in sensor 1
            print("Sum of both angles is", abs(dir1) + abs(dir2) )
            if abs(dir1_orig - dir2in1) <= 45 :#and abs(dir1_orig - dir2in1) >= 0:
                sound_received_once= True
                angle_avg = ((dir1_orig + dir2in1)/2)
                print("angle1, angle2 and angle2in1 are: ",dir1_orig, dir2, dir2in1)
                print("angle avg is:", angle_avg)
                angle_avg = angle_avg - 90  # 90 is the diff between sensor 1 and robot base
                # if angle_avg > 180:
                #     angle_avg = angle_avg - 360
                if angle_avg<0:
                    angle_avg= 360-angle_avg
                print("Goal couldn't be ound but average angle of goal direction is:", angle_avg)
                # Lets convert angle of rotation to quaternion
                # angle_avg= math.radians(angle_avg)
                if angle_avg<360 and angle_avg>-180:
                    pub.publish(angle_avg)
