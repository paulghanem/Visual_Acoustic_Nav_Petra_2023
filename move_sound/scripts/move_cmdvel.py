#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math

roll = pitch = yaw = 0.0
target = 1.57
kp=0.3

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)

def callback(data):
    global target
    angle= data.data
    angle=angle+89+1
    if angle>180:
        angle=angle-360
    target=angle

rospy.init_node('rotate_robot')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
# sub2= rospy.Subscriber('/sound_direction', Int32, callback)
pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10)
command =Twist()

while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    target_rad = target*math.pi/180
    command.angular.z = kp * (target_rad-yaw)
    # if abs(target_rad-yaw) <0.09:
    #     command.angular.z=0.0
    #     rospy.loginfo("angular distance lss than threshold")
    #     break
    pub.publish(command)
    print("taeget={} current:{}", target,yaw)
    r.sleep()