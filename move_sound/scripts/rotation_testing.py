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
import tf, tf_conversions, tf2_ros, tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from move_sound.msg import HSE_Object, HSE_Objects
import cv2
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from numpy.lib.stride_tricks import as_strided
import imutils
from collections import Counter

def rotate_scan():
    client.wait_for_server()
    client.cancel_all_goals()
    angle_rot= math.radians(180)
    quat=quaternion_from_euler(0, 0, angle_rot)
    tf_bufferR = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_bufferR)
    pose_stamped3 = tf2_geometry_msgs.PoseStamped()
    pose_stamped3.pose.position.x = 0
    pose_stamped3.pose.position.y = 0
    pose_stamped3.pose.position.z = 0
    pose_stamped3.pose.orientation.x = quat[0]
    pose_stamped3.pose.orientation.y = quat[1]
    pose_stamped3.pose.orientation.z = quat[2]
    pose_stamped3.pose.orientation.w = quat[3]
    pose_stamped3.header.frame_id = "base_link"
    pose_stamped3.header.stamp = rospy.Time(0)
    curr_posi = tf_bufferR.transform(pose_stamped3, "map", rospy.Duration(1))
                
    # rotate command
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = curr_posi.pose.position.x
    goal.target_pose.pose.position.y = curr_posi.pose.position.y
    goal.target_pose.pose.orientation.z = curr_posi.pose.orientation.z
    goal.target_pose.pose.orientation.w = curr_posi.pose.orientation.w
    print("Rotating to scan env")
    client.cancel_all_goals()
    client.send_goal(goal)
    client.wait_for_result()
    client.get_result()

def foundMummy(found_at):
    print("Human Found at",found_at)

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.Subscriber("/Visual_Goal", PoseStamped, foundMummy)
        # rotate_scan()
        # Initial_goal  gaussian_max_goal
        
        rospy.spin()
#        for i in goal_rec:
#            client.wait_for_server()
#            client.cancel_all_goals()
#            goal= movebase_client(i)
#            client.send_goal(goal)
#            print("Goal Sent", goal)
#            client.wait_for_result(rospy.Duration())
#            print("waiting 1 sec")
#            client.get_result()
        #result = movebase_client()
        #if result:
        #    rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
