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


# dev = usb.core.find(idVendor=0x2886,idProduct=0x0018, bcdDevice=3.00)
devices = tuple(usb.core.find(
    find_all=True, idVendor=0x2886, idProduct=0x0018))
for i in range(len(devices)):
    print(devices[i].bus, devices[i].address)

##############################################
dev1 = devices[0]
dev2 = devices[1]
print(dev2)
print("==================================================================")
print(dev1)
# dev = usb.core.find(find_all=True)
# for d in dev:
#     print(usb.util.get_string(d,d.iManufacturer))
#     print(usb.util.get_string(d,d.iProduct))
#     print(d.idProduct,d.idVendor)


if len(devices) > 1:
    Mic_tuning1 = Tuning(dev1)
    Mic_tuning2 = Tuning(dev2)
    dir = Int32MultiArray()
    rospy.init_node('doa_node')
    pub = rospy.Publisher('Initial_goal', PoseStamped, queue_size=10)
    pub2 = rospy.Publisher('blind_rot_sig', PoseStamped, queue_size=10)
    # rospy.loginfo(Mic_tuning.direction)
    counter=0
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        vad1 = Mic_tuning1.is_voice()
        vad2 = Mic_tuning2.is_voice()
        # rospy.loginfo(vad1)
        # rospy.loginfo(vad2)
        if vad1 == 1 and vad2 == 1:
            dir1 = Mic_tuning1.direction
            dir1_orig = dir1
            if dir1 > 180:
                dir1 = dir1-360
            dir2 = Mic_tuning2.direction
            if dir2 > 180:
                dir2 = dir2-360
            dir2in1 = dir2 + 180  # sensor 2 angle in sensor 1
            # dir_list=[]
            # dir_list.append(dir1)
            # dir_list.append(dir2)

            base = 0.2286
            point = PoseStamped()
            print("dir1,2 are: =====", dir1, dir2)
            print("dir1_orig , dir2in1 are ", dir1_orig, dir2in1)
            print("dir1_orig - dir2in1= ", (dir1_orig - dir2in1))
            # This part is for calculating the distance of midpont of both sensors to the detected sound source.
            # Lets add some noise to both values obtained
            print("Sum of both angles is", abs(dir1) + abs(dir2) )
            
#            if dir1 >= 0 && dir2 >=0:
                # Rotate the robot by mean degrees
#            noise = np.random.normal(0,4.5,100)
#            print(np.round(noise))
##            if dir1<0:
##                direction1= dir1 -noise
##            else:
##                direction1= dir1+ noise
##            if dir2<0:
##                direction2= dir2 - noise
##            else:
##                direction2= dir2 + noise
#            direction1= dir1+noise
#            direction2= dir2 +noise
#            print("direction1", np.unique(np.round(direction1)))
#            print("direction2", np.unique(np.round(direction2)))
#            
#            print("SIZES Of dir1, dir2", direction1.shape, direction2.shape)
#            print("max and min values", np.max(direction1), np.min(direction1))
##            print("Dir1,2 are", direction1,direction2)
#            #make combinations
#            angle_combo= np.array(np.meshgrid(direction1, direction2)).T.reshape(-1,2)
##            print(angle_combo)
#            sum1= np.sum(np.abs(angle_combo), axis=1)
##            print("sum is", sum1)
#            angle_combo= np.column_stack((angle_combo, sum1))
##            print("shape of angle_combo", angle_combo.shape)
#            angle_combo= angle_combo[angle_combo[:,2]<180]
##            print("Angle combos are: ", angle_combo)
#            angle3 = 180 - (np.abs(angle_combo[:,0]) + np.abs(angle_combo[:,1]))
##            print("Sum of all 3 angles", np.sum(np.column_stack((angle_combo[:,2], angle3)), axis=1))
#            angle3=np.radians(angle3)
#            angle_combo=np.radians(angle_combo)
#            angle_combo= angle_combo[angle_combo[:,2]>0]
#            dist1 = np.divide(np.sin(np.abs(angle_combo[:,1])) , np.sin(angle3))
#            dist1 = dist1 * 0.2286
#            dist2 = np.divide(np.sin(np.abs(angle_combo[:,0])), np.sin(angle3))
#            dist2 = dist2 * 0.2286
##            print("max distance is:", np.max(dist1) ,"at angles: ",np.degrees(angle_combo[np.argmax(dist1),:]))
#            xcoord = dist1*np.cos(angle_combo[:,0])
#            ycoord = dist1*np.sin(angle_combo[:,0])
#            xcoord2 = dist2*np.cos(angle_combo[:,1])
#            ycoord2 = dist2*np.sin(angle_combo[:,1])
#            print("Mean value point for sensor 1 is:", np.mean(xcoord), np.mean(ycoord))
#            print("Median value point wrt sensor 1 is:", np.median(xcoord), np.median(ycoord))
#            print("Mean value point wrt sensor2 is:", np.mean(xcoord2), np.mean(ycoord2))
#            print("Median value point wrt sensor 2 is:", np.median(xcoord2), np.median(ycoord2))
#            coords= np.column_stack((xcoord,ycoord))
#            
#            from matplotlib import pyplot as plt
#            x = xcoord
#            y = ycoord
#            plt.grid()
#            plt.scatter(x, y)
#            plt.show()
            
            if dir1*dir2 < 0:  #(abs(dir1) + abs(dir2)) < 180 and
                noise = np.random.normal(0,4.5,100)
                print(np.round(noise))
#                if dir1<0:
#                    direction1= dir1 -noise
#                else:
#                    direction1= dir1+ noise
#                if dir2<0:
#                    direction2= dir2 - noise
#                else:
#                    direction2= dir2 + noise
                direction1= dir1+noise
                direction2= dir2 +noise
                print("direction1", np.unique(np.round(direction1)))
                print("direction2", np.unique(np.round(direction2)))
            
                print("SIZES Of dir1, dir2", direction1.shape, direction2.shape)
                print("max and min values", np.max(direction1), np.min(direction1))
#               print("Dir1,2 are", direction1,direction2)
                #make combinations
                angle_combo= np.array(np.meshgrid(direction1, direction2)).T.reshape(-1,2)
#                print(angle_combo)
                sum1= np.sum(np.abs(angle_combo), axis=1)
#               print("sum is", sum1)
                angle_combo= np.column_stack((angle_combo, sum1))
#               print("shape of angle_combo", angle_combo.shape)
                angle_combo= angle_combo[angle_combo[:,2]<180]
#                print("Angle combos are: ", angle_combo)
                angle3 = 180 - (np.abs(angle_combo[:,0]) + np.abs(angle_combo[:,1]))
#                print("Sum of all 3 angles", np.sum(np.column_stack((angle_combo[:,2], angle3)), axis=1))
                angle3=np.radians(angle3)
                angle_combo=np.radians(angle_combo)
                angle_combo= angle_combo[angle_combo[:,2]>0]
                dist1 = np.divide(np.sin(np.abs(angle_combo[:,1])) , np.sin(angle3))
                dist1 = dist1 * 0.2286
                dist2 = np.divide(np.sin(np.abs(angle_combo[:,0])), np.sin(angle3))
                dist2 = dist2 * 0.2286
                print("mean of difference of mean of both distances",np.mean(dist1-dist2))
                print("mean of differecnce of median",np.median(dist1-dist2))
#                print("max distance is:", np.max(dist1) ,"at angles: ",np.degrees(angle_combo[np.argmax(dist1),:]))
                xcoord = dist1*np.cos(angle_combo[:,0])
                ycoord = dist1*np.sin(angle_combo[:,0])
                xcoord2 = dist2*np.cos(angle_combo[:,1])
                ycoord2 = dist2*np.sin(angle_combo[:,1])
                
#                angle3 = 180 - (abs(dir1) + abs(dir2))
#                dist1 = 0.2286*math.sin(math.radians(abs(dir2))) / math.sin(math.radians(angle3))
#                xcoord = dist1*math.cos(math.radians(dir1))
#                ycoord = dist1*math.sin(math.radians(dir1))
#                print(" Goal found with directions", dir1, dir2, "and its x and y values are: ", xcoord, ycoord)
#                z_orientation= quaternion_from_euler(0, 0, dir1)

                my_pose = Pose()
                my_pose.position.x = np.median(xcoord)
                my_pose.position.y = np.median(ycoord)
                my_pose.position.z = 0.0
###                my_pose.orientation.x = 0.0
###                my_pose.orientation.y = 0.0
###                my_pose.orientation.z = z_orientation[2]
###                my_pose.orientation.w = z_orientation[3]

#                # **Assuming /tf2 topic is being broadcasted
                tf_buffer = tf2_ros.Buffer()
                listener = tf2_ros.TransformListener(tf_buffer)
                pose_stamped = tf2_geometry_msgs.PoseStamped()
                pose_stamped.pose = my_pose
                pose_stamped.header.frame_id = "sensor1"
                pose_stamped.header.stamp = rospy.Time(0)
                point = tf_buffer.transform(
                    pose_stamped, "map", rospy.Duration(1))
                
                try:
                    print("Goal x,y found and it is: ", point)
                    pub.publish(point)
                    time.sleep(0.5)
                except KeyboardInterrupt:
                    break
            elif abs(dir1_orig - dir2in1) <= 30 and abs(dir1_orig - dir2in1) >= 0:
                angle_avg = ((dir1_orig + dir2in1)/2)
                print("angle1, angle2 and angle2in1 are: ",dir1_orig, dir2, dir2in1)
                print("angle avg is:", angle_avg)
                angle_avg = angle_avg - 90  # 90 is the diff between sensor 1 and robot base
                if angle_avg > 180:
                    angle_avg = angle_avg - 360
                print("Goal couldn't be ound but average angle of goal direction is:", angle_avg)

                # Lets convert angle of rotation to quaternion
                angle_avg= math.radians(angle_avg)
                quat=quaternion_from_euler(0, 0, angle_avg)
                tf_buffer2 = tf2_ros.Buffer()
                listener = tf2_ros.TransformListener(tf_buffer2)
                pose_stamped2 = tf2_geometry_msgs.PoseStamped()
                pose_stamped2.pose.position.x = 0
                pose_stamped2.pose.position.y = 0
                pose_stamped2.pose.position.z = 0
                pose_stamped2.pose.orientation.x =quat[0]
                pose_stamped2.pose.orientation.y =quat[1]
                pose_stamped2.pose.orientation.z =quat[2]
                pose_stamped2.pose.orientation.w =quat[3]
                pose_stamped2.header.frame_id = "base_link"
                pose_stamped2.header.stamp = rospy.Time(0)
                point2 = tf_buffer2.transform(
                	    pose_stamped2, "map", rospy.Duration(1))
                
                if counter==0:
                    # rotate command
                    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                    client.wait_for_server()
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = point2.pose.position.x
                    goal.target_pose.pose.position.y = point2.pose.position.y
#                    goal.target_pose.pose.orientation.x = point2.pose.orientation.x
#                    goal.target_pose.pose.orientation.y = point2.pose.orientation.y
                    goal.target_pose.pose.orientation.z = point2.pose.orientation.z
                    goal.target_pose.pose.orientation.w = point2.pose.orientation.w
                    print("Directional goal is",goal)
                    client.cancel_all_goals()
                    client.send_goal(goal)
                    client.wait_for_result()
                    client.get_result()
                    
                    # wait = client.wait_for_result()
                    # if not wait:
                        # rospy.logerr("Action server not available!")
                        # rospy.signal_shutdown("Action server not available!")
                    # else:
                    #     print("goal reached")
                else:
                    try:
                        print("Publishing rotation for 2nd time")
                        point2.pose.orientation.z= quat[2] # Orientation should be wrt robot and not wrt map unless you wanna command move_base goal| goal position however should be wrt map
                        point2.pose.orientation.w= quat[3] # Orientation wrt robot and not wrt map becoz we update pixels from robot curr posi and curr orientation and not map orientation
                        pub2.publish(point2)
                        print(point2)                       
                        time.sleep(0.5)
                    except KeyboardInterrupt:
                        break               
