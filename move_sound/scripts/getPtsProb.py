#!/usr/bin/env python3

from cmath import sin
from tuning import Tuning
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray, Float32
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

map_=np.zeros((10,10), np.int8)
origin_x=0
origin_y=0
resolution=0.05
areas= np.empty((0,4))
# img =np.zeros((10,10), np.int8)
publish_now= False
# map_div_ind= np.empty((0,2), int)
human_found_at = PointStamped()
human_found = False
ready=False
prob2=np.array((1))
sound_received_once= False
points_length= 0
prob2_update=False

areas= np.vstack((areas,[0,0,0,0]))
areas= np.vstack((areas,[0,0, 2,0]))
areas= np.vstack((areas,[0,0, 3,-2]))
areas= np.vstack((areas,[0,0,2.5,-4]))
#areas= np.vstack((areas,[0,0,2,-4]))
areas= np.vstack((areas,[0,0,6,-3]))
#areas= [[0,0,0,0][0,0, 2,0][0,0, 4,0][0,0,3,-2][0,0,0.5,-3.5]]
points_length= len(areas)
prob2= np.ones(areas.shape[0])
prob2[0]=0.0
prob2=np.divide(prob2, np.sum(prob2))
prob2_update=True
ready= True
print("ready set to True")
print("Areas finally are:", areas)
print("Prob2 created by setPts is:", prob2)



def occupancygrid_to_numpy(msg):  # Converts map to numpy
    global map_, origin_x, origin_y, resolution
    map_ = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height))
    unq, counts = np.unique(map_, return_counts=True)
    print("unique_values in input map are: ", unq, counts)
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    # resolution= msg.info.resolution
    print("originx, y, resolution are: ",origin_x, origin_y, resolution )
    print("received map values okay to proceed")


def numpy_to_occupancy_grid(arr, info=None): # Converts numpy matrix to map
    if not len(arr.shape) == 2:
        raise TypeError('Array must be 2D')
    if not arr.dtype == np.int8:
        raise TypeError('Array must be of int8s')
    grid = OccupancyGrid()
    grid.data = arr.ravel()
    grid.info = info or MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]
    grid.info.resolution= 0.05
    return grid


def point2pixel(point1):  # converts x,y map frame to pixels    
    x_cell = np.abs(np.round((point1.point.x - origin_x)/resolution))
    y_cell = np.abs(np.round((point1.point.y - origin_y)/resolution))
    x_cell = int(x_cell)
    y_cell = int(y_cell)
    print("point2pixel returns xcell and ycell are: ", [x_cell,y_cell])
    print("in row major index:", x_cell + (y_cell)*4000)
    return [x_cell,y_cell]


# def setPoints(point1):  # Takes input points from map selected by human
#     global areas, publish_now, prob2, points_length, ready
#     #Convert points to pixel ind on map
#     ind = point2pixel(point1)
#     print("MAp value of new point", map_[int(ind[0]),int(ind[1])])
#     if map_[int(ind[0]),int(ind[1])]!=0:
#         if len(areas)<1:
#             tf_bufferS = tf2_ros.Buffer()
#             listenerS = tf2_ros.TransformListener(tf_bufferS)
#             pose_stamped4 = tf2_geometry_msgs.PoseStamped()
#             pose_stamped4.pose.position.x = 0
#             pose_stamped4.pose.position.y = 0
#             # pose_stamped4.point.z = 0
#             pose_stamped4.header.frame_id = "base_link"
#             pose_stamped4.header.stamp = rospy.Time(0)
#             point = tf_bufferS.transform( pose_stamped4, "map", rospy.Duration(1))
#             point2= PointStamped()
#             point2.point.x= point.pose.position.x
#             point2.point.y= point.pose.position.y
#             #calculate current position on map
#             # point3= geometry_msgs.PointStamped
#             ind=point2pixel(point2)
#             ind= np.hstack((ind,[point2.point.x, point2.point.y]))
#             print("ind becomes", ind)
#             print("areas shape is",areas.shape)
#             areas=np.vstack((areas,ind))
#             print("Robots position inserted in areas: ", areas)
#             ind=point2pixel(point1) # Inserting the clicked point
#             ind=np.hstack((ind,[point1.point.x, point1.point.y]))
#             areas= np.vstack((areas,ind))
#             print("areas now are",areas)
#         else:
#             ind=np.hstack((ind,[point1.point.x, point1.point.y]))
#             areas= np.vstack((areas,ind))
#             print("areas now are",areas)
#             print("Latest value of cell in map is: ",map_[int(ind[0]),int(ind[1])])
#     # Start position of robot has 0 prob of human present 
#     elif map_[int(ind[0]),int(ind[1])]==0: # Last point should be from outside scanned map
#         points_length= len(areas)
#         print("Length of areas array is: ", areas)
#         prob2= np.ones(areas.shape[0])
#         prob2[0]=0.0
#         prob2=np.divide(prob2, np.sum(prob2))
#         prob2_update=True
#         ready= True
#         print("ready set to True")
#         print("Areas finally are:", areas)
#         print("Prob2 created by setPts is:", prob2)
#         #Lets rotate the robot to scan environment and check for human face/body

def angle_from_base():
    global areas
    points_wrt_base=np.empty((0,2))
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_stamped = tf2_geometry_msgs.PointStamped()
    for i in range(len(areas[:,0])):
        pose_stamped.point.x = areas[i][2]
        pose_stamped.point.y = areas[i][3]
        pose_stamped.point.z = 0
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time(0)
        point2 = tf_buffer.transform( pose_stamped, "base_link", rospy.Duration(1))
        points_wrt_base= np.vstack((points_wrt_base, [point2.point.x,point2.point.y]))
    angles_wrt_base = np.arctan2(points_wrt_base[:,-1], points_wrt_base[:,0]) *180 /np.pi
    angles_wrt_base = np.mod(angles_wrt_base, 360)
    return angles_wrt_base

def probs(direc): # direction sound signal edits prob2
    global prob2, ready, human_found
    print("Direction received and ready status =", ready)
    direction= direc.data
    print("direction received is",direction)
    print(human_found)
    if ready and not human_found:
        # prob=np.ones((areas.shape), float)
        angles_wrt_base= angle_from_base()   
        print("Angles wrt base_link are: ", angles_wrt_base)    
        #Calculate probability for each array angle
        direction = np.round(direction)#.astype(int)
        arr1 = np.random.normal(loc=direction, scale= 13 , size=4000)
        arr1 = np.round(arr1)
        arr1 = arr1%359  # Circulating all values into continous angles of range(0-360)
        arr1 = arr1.astype(int)
        ind_arr = Counter(arr1)
        prob = np.ones(360,dtype='int32')
        for ele in ind_arr:    
            prob[ele] = ind_arr[ele] + 1
        # prob = np.divide(prob, np.sum(prob))
        dir = np.round(angles_wrt_base)
        dir=dir%359
        dir= dir.astype(int)
        print("Dir calclated", prob)
        prob2 = prob[dir]
        print("probability of 4 given angles is", prob2)
        prob2[0]=0 # assigning first prob =0 because that is where the robot starts
        prob2 = np.divide(prob2, np.sum(prob2)) # Normalize
        prob2_update=True
        print("probability after normalization is:, ", prob2)
        print("Sum check:", np.sum(prob2))
    else:
        print("NOT READY OR HUMAN ALREADY FOUND")
     
def foundMummy(data): # This method is for dealing with human found module. 
    # Gets called regularly and whenever sees human just cancels all goals and sets human goal
    global human_found, human_found_at, areas, prob2, prob2_update
    human_found_at.point.x = data.pose.position.x
    human_found_at.point.y = data.pose.position.y
    human_found=True
    if human_found :
        # Later we need to add Find random point at 1 meter distance from goal
        # client.wait_for_server()
        # client.cancel_all_goals()
        if len(areas)==points_length:
            dist_from_points= np.sqrt( np.square(areas[:,2]- human_found_at.point.x) + np.square(areas[:,3]- human_found_at.point.y))
            row= np.argwhere(dist_from_points<0.4)
            print("Row value is##########################",row)
            if row:
                # SInce point already present update probability
                prob2=prob2-prob2 # make all values 0 and location value 1
                prob2[row]= 1
                print(" point was already present Updated prob2 after found human", prob2)
                prob2_update=True
            else:
                # Add new point to areas since point not already present
                ind= point2pixel(human_found_at)
                ind= np.hstack((ind,[human_found_at.point.x, human_found_at.point.y]))
                areas=np.vstack((areas,ind))
                prob2=prob2-prob2
                print("Length of prob2 before adding pt", len(prob2))
                prob2=np.hstack((prob2, 1.0))
                print("Added location to areas and Updated prob2 after found human", prob2)
                prob2_update=True
        elif len(areas)== points_length+1:
            # Since once the human location has been added earlier, we are just updating it
            ind= point2pixel(human_found_at)
            ind= np.hstack((ind,[human_found_at.point.x, human_found_at.point.y]))
            areas[-1,:]= ind
            prob2=prob2-prob2
            print("Length of prob2 before adding pt", len(prob2))
            prob2[-1]=1.0
            prob2_update=True
    
def checkDistFromGoal(odom_data):  # Keeps getting called since odom is published at constant rate
    # keeps checking which points have been traversed by robot while no human was found and keeps updating probs2
    global prob2, ready, human_found, prob2_update
    # print("received from odom,", odom_data.pose.pose.position.x)
    ##Here it was
    print("Entered checkDist prob2_update status is", prob2_update)
    curr_pos1 = tf2_geometry_msgs.PoseStamped()
    curr_pos1.pose.position.x = 0
    curr_pos1.pose.position.y = 0#areas[i][3]
    # curr_pos1.pose.position.z = 0
    curr_pos1.header.frame_id = "base_link"
    curr_pos1.header.stamp = rospy.Time(0)
    try:
        curr_pos_map = tf_buffer1.transform( curr_pos1, "map", rospy.Duration(4.0))
        # print("transform correct")
    except Exception as e:
            rospy.logerr(e)
    # print("transform correct")
    threshold=1.0
    if ready and not prob2_update:
        angles_wrt_base = angle_from_base()
        angles_wrt_base= np.round(angles_wrt_base).astype(int)
        dist_from_points= np.sqrt( np.square(areas[:,2]- curr_pos_map.pose.position.x) + np.square(areas[:,3]- curr_pos_map.pose.position.y))
        if not human_found:
            pts_near_ind= np.argwhere(dist_from_points<threshold)
            angle_in_scan_range_ind= np.argwhere((angles_wrt_base<35) | (angles_wrt_base>325))
            pts_to_be_nulled_ind= np.intersect1d(pts_near_ind, angle_in_scan_range_ind) 
            prob2[pts_to_be_nulled_ind]= 0.0 # For now threshold distance is 2 meters 
            prob2= np.divide(prob2, np.sum(prob2))
            print("prob2 value is", prob2)
            prob2_update=True

# def rotate_scan():
#     client.wait_for_server()
#     client.cancel_all_goals()
#     angle_rot= math.radians(360)
#     quat=quaternion_from_euler(0, 0, angle_rot)
#     tf_bufferR = tf2_ros.Buffer()
#     listenerR = tf2_ros.TransformListener(tf_bufferR)
#     pose_stamped3 = tf2_geometry_msgs.PoseStamped()
#     pose_stamped3.pose.position.x = 0
#     pose_stamped3.pose.position.y = 0
#     pose_stamped3.pose.position.z = 0
#     pose_stamped3.pose.orientation.x = quat[0]
#     pose_stamped3.pose.orientation.y = quat[1]
#     pose_stamped3.pose.orientation.z = quat[2]
#     pose_stamped3.pose.orientation.w = quat[3]
#     pose_stamped3.header.frame_id = "base_link"
#     pose_stamped3.header.stamp = rospy.Time(0)
#     curr_posi = tf_bufferR.transform(pose_stamped3, "map", rospy.Duration(1))
                
#     # rotate command
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = curr_posi.pose.position.x
#     goal.target_pose.pose.position.y = curr_posi.pose.position.y
#     goal.target_pose.pose.orientation.z = curr_posi.pose.orientation.z
#     goal.target_pose.pose.orientation.w = curr_posi.pose.orientation.w
#     print("Rotating to scan env")
#     client.cancel_all_goals()
#     client.send_goal(goal)
#     client.wait_for_result()
#     client.get_result()

# def sendgoal(goal_):
#     client.wait_for_server()
#     client.cancel_all_goals()
#     tf_buffer2 = tf2_ros.Buffer()
#     listener2 = tf2_ros.TransformListener(tf_buffer2)
#     curr_pos = tf2_geometry_msgs.PoseStamped()
#     curr_pos.pose.position.x = goal_[0]#goal_.pose.position.x
#     curr_pos.pose.position.y = goal_[1]#goal_.pose.position.y
#     curr_pos.pose.position.z = 0
#     curr_pos.header.frame_id = "map"
#     curr_pos.header.stamp = rospy.Time(0)
#     point3 = tf_buffer2.transform(curr_pos, "base_link", rospy.Duration(1))
# #    point3= tf_buffer2.transform("")
# #    After converting goal from map to base_link frame get rotation wrt robot frame
#     rotation_needed= math.atan2(point3.pose.position.y, point3.pose.position.x)
#     rotation_min= rotation_needed
#     rotation_min=math.radians(rotation_min)
#     qu= quaternion_from_euler(0.0,0.0,rotation_min)
#     point3.pose.orientation.z= qu[2]
#     point3.pose.orientation.w= qu[3]
#     # Convert above point from base_link to map frame and provide to goal otherwise it gives error/warning "goal must be passed in map frame for this planner"
# #    print("rotation in base_link frame is", np.degrees(euler_from_quaternion([0.0,0.0,point3.pose.orientation.z, point3.pose.orientation.w])))  
#     # tf_buffer3 = tf2_ros.Buffer()
#     # listener3 = tf2_ros.TransformListener(tf_buffer3)
#     cur_pose2= tf2_geometry_msgs.PoseStamped()
#     cur_pose2.pose.position.x= point3.pose.position.x
#     cur_pose2.pose.position.y= point3.pose.position.y
#     cur_pose2.pose.orientation.z= point3.pose.orientation.z
#     cur_pose2.pose.orientation.w= point3.pose.orientation.w
#     cur_pose2.header.frame_id = "base_link"
#     cur_pose2.header.stamp = rospy.Time(0)
#     point4 = tf_buffer2.transform( cur_pose2, "map", rospy.Duration(1))
# #    print("rotation in map frame is:", np.degrees(euler_from_quaternion([0.0,0.0,point4.pose.orientation.z, point4.pose.orientation.w])))
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = point4.pose.position.x
#     goal.target_pose.pose.position.y = point4.pose.position.y
#     goal.target_pose.pose.orientation.z= point4.pose.orientation.z
#     goal.target_pose.pose.orientation.w = point4.pose.orientation.w

#     client.send_goal(goal)
#     print("Goal Sent", goal)
#     client.wait_for_result(rospy.Duration(10))
#     print("waiting 1 sec")
#     client.get_result()

####################################################################
rospy.init_node('doa_node2')
tf_buffer1 = tf2_ros.Buffer()
listener1 = tf2_ros.TransformListener(tf_buffer1)
# client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
pub= rospy.Publisher("Initial_goal", PoseStamped, queue_size=10)
#rospy.Subscriber("/map", OccupancyGrid, occupancygrid_to_numpy)
#rospy.Subscriber("/clicked_point", PointStamped, setPoints)
rospy.Subscriber('/trigger_checkDist', Int32, checkDistFromGoal)
rospy.Subscriber("/Visual_Goal", PoseStamped, foundMummy)
#rospy.Subscriber("/sound_angle", Float32, probs)

while not rospy.is_shutdown():
    if ready and prob2_update and len(prob2)>=points_length and points_length>0 :
        goal_rn= PoseStamped()
        print("Areas value are", areas)
        print("prob2 values are", prob2)
        goal_rn.pose.position.x= areas[np.argmax(prob2),2]
        goal_rn.pose.position.y= areas[np.argmax(prob2),3]
        pub.publish(goal_rn)
        print("Sending new goals",goal_rn)
        prob2_update=False
rospy.spin()
##########################################################################
