#!/usr/bin/env python3

from cmath import sin
from tuning import Tuning
from std_msgs.msg import Int32, Int32MultiArray
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

# def checkDistFromGoal(odom_data):  # Keeps getting called since odom is published at constant rate
#     # keeps checking which points have been traversed by robot while no human was found and keeps updating probs2
#     # global prob2, ready, human_found, prob2_update
#     # print("received from odom,", odom_data.pose.pose.position.x)
#     tf_buffer1 = tf2_ros.Buffer()
#     listener1 = tf2_ros.TransformListener(tf_buffer1)
#     curr_pos1 = tf2_geometry_msgs.PoseStamped()
#     curr_pos1.pose.position.x = 0
#     curr_pos1.pose.position.y = 0#areas[i][3]
#     # curr_pos1.pose.position.z = 0
#     curr_pos1.header.frame_id = "base_link"
#     curr_pos1.header.stamp = rospy.Time(0)
#     try:
#         curr_pos_map = tf_buffer1.transform( curr_pos1, "map", rospy.Duration(2.5))
#         print("transform correct")
#     except Exception as e:
#             rospy.logerr(e)

if __name__ == '__main__':

    rospy.init_node('chec_dist')
    rate = rospy.Rate(0.5)
    pub = rospy.Publisher('trigger_checkDist', Int32, queue_size=1)
    while not rospy.is_shutdown():
        pub.publish(1)
        print("published")
        rate.sleep()