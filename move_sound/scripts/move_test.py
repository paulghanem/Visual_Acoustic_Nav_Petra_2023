#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import time
import tf
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

goal_rec= []

def sendgoal(goal_):
    client.wait_for_server()
    client.cancel_all_goals()

    
    tf_buffer11 = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer11)
    curr_pos = tf2_geometry_msgs.PoseStamped()
    curr_pos.pose.position.x = goal_[0]#goal_.pose.position.x
    curr_pos.pose.position.y = goal_[1]#goal_.pose.position.y
    curr_pos.pose.position.z = 0
    curr_pos.header.frame_id = "map"
    curr_pos.header.stamp = rospy.Time(0)
    point3 = tf_buffer11.transform(curr_pos, "base_link", rospy.Duration(1))
#    point3= tf_buffer11.transform("")
#    After converting goal from map to base_link frame get rotation wrt robot frame
    rotation_needed= math.atan2(point3.pose.position.y, point3.pose.position.x)
#    print("atan2 in base_link frame", math.degrees(rotation_needed))
    rotation_min=0
    rotation_needed= math.degrees(rotation_needed)
    if rotation_needed>180:
        rotation_min=rotation_needed%-180
    elif rotation_needed<-180:
        rotation_min= rotation_needed%180
    else:
        rotation_min= rotation_needed
    print("rotation needed is ", rotation_min)
    rotation_min=math.radians(rotation_min)
    qu= quaternion_from_euler(0.0,0.0,rotation_min)
    print("")
    point3.pose.orientation.z= qu[2]
    point3.pose.orientation.w= qu[3]
    # Convert above point from base_link to map frame and provide to goal otherwise it gives error/warning "goal must be passed in map frame for this planner"
#    print("rotation in base_link frame is", np.degrees(euler_from_quaternion([0.0,0.0,point3.pose.orientation.z, point3.pose.orientation.w])))
    
    
#    tf_buffer3 = tf2_ros.Buffer()
#    listener = tf2_ros.TransformListener(tf_buffer3)
    cur_pose2= tf2_geometry_msgs.PoseStamped()
    cur_pose2.pose.position.x= point3.pose.position.x
    cur_pose2.pose.position.y= point3.pose.position.y
    cur_pose2.pose.orientation.z= point3.pose.orientation.z
    cur_pose2.pose.orientation.w= point3.pose.orientation.w
    cur_pose2.header.frame_id = "base_link"
    cur_pose2.header.stamp = rospy.Time(0)
    point4 = tf_buffer11.transform(cur_pose2, "map", rospy.Duration(1))
#    print("rotation in map frame is:", np.degrees(euler_from_quaternion([0.0,0.0,point4.pose.orientation.z, point4.pose.orientation.w])))
    
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point4.pose.position.x
    goal.target_pose.pose.position.y = point4.pose.position.y
    goal.target_pose.pose.orientation.z= point4.pose.orientation.z
    goal.target_pose.pose.orientation.w = point4.pose.orientation.w
    
    
    client.send_goal(goal)
    print("Goal Sent", goal)
    client.wait_for_result(rospy.Duration(10))
    print("waiting 1 sec")
    client.get_result()
    
    #client.cancel_all_goals()
    #if not wait:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")
    #else:
    #    return client.get_result()

def movebase_client(goals): #goals
    global goal_rec
    
    x1= goals.pose.position.x  #goal[0] #
    y1= goals.pose.position.y   #goal[1] 
    print(x1,y1)
#    goal_rec.append([x1,y1])
#    print("goal_rec now is", [x1,y1], goal_rec[-1])
#    print("sending goal now", [x1,y1],goal_rec[-1])
    client.wait_for_server()
#    sendgoal([x1,y1]) # goal_rec[-1]
    if len(goal_rec)>0:
        print("last value square in goal_rec is:", goal_rec[-1][0]**2, goal_rec[-1][1]**2)
        print("x1 and y1 square is", x1**2, y1**2 )
        print("diff of sq is:", (x1**2-goal_rec[-1][0]**2), (y1**2 - goal_rec[-1][1]**2)) 
        print("sqrt of difference is",math.sqrt((x1-goal_rec[-1][0])**2+ (y1 - goal_rec[-1][1])**2 ))
        if math.sqrt((x1-goal_rec[-1][0])**2+ (y1 - goal_rec[-1][1])**2 ) > 0.3:
            goal_rec.append([x1,y1])
            print("goal_rec now is", goal_rec)
            print("sending goal now", goal_rec[-1])
            client.wait_for_server()
            sendgoal(goal_rec[-1])
        else:
            print("No new goal")
            
    else:
        goal_rec.append([x1,y1])
        print("goal_rec now is", goal_rec)
        print("sending goal now", goal_rec[-1])
        client.wait_for_server()
        sendgoal(goal_rec[-1])
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')     
#        goal_rec=[[2,0],[4,0],[6,0],[6.5,0]],[0.5,-3.25]]#,[0.0, 0.5], [1,1]
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        print("Im here")
        rospy.Subscriber("/Initial_goal", PoseStamped, movebase_client) # Initial_goal  gaussian_max_goal
        
        rospy.spin()
#        for i in goal_rec:
#            client.wait_for_server()
#            client.cancel_all_goals()
#            print("movebase client called")
#            goal= movebase_client(i)
##            client.send_goal(goal)
##            print("Goal Sent", goal)
##            client.wait_for_result(rospy.Duration())
#            print("waiting 1 sec")
#            client.get_result()
#        result = movebase_client()
#        if result:
#            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
