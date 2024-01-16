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
from move_sound.msg import HSE_Object, HSE_Objects
import cv2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from numpy.lib.stride_tricks import as_strided
import imutils

map_=np.zeros((10,10), np.int8)
origin_x=0
origin_y=0
resolution=0.05
areas= np.empty((0,2), int)
img =np.zeros((10,10), np.int8)
publish_now= False
map_div_ind= np.empty((0,2), int)

def occupancygrid_to_numpy(msg):
    global map_, origin_x, origin_y, resolution

    map_ = np.asarray(msg.data, dtype=np.uint8).reshape((msg.info.width, msg.info.height))
    unq, counts= np.unique(map_, return_counts=True)
    print("unique_values in input map are: ", unq, counts)
    origin_x= msg.info.origin.position.x
    origin_y= msg.info.origin.position.y
    # resolution= msg.info.resolution
    print("originx, y, resolution are: ",origin_x, origin_y, resolution )
    print("received map values okay to proceed")



def numpy_to_occupancy_grid(arr, info=None):
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
    print(grid.data[-1])
    return grid
    
def floor0(value):
    if (value < 0.0):
        return np.ceil(value )
    else:
        return np.floor(value)

def point2pixel(point1):
    
    x_cell = np.abs(np.round((point1.point.x - origin_x)/resolution))
    y_cell = np.abs(np.round((point1.point.y - origin_y)/resolution))
    x_cell = int(x_cell)
    y_cell = int(y_cell)
    print("point2pixel returns xcell and ycell are: ", [x_cell,y_cell])
    print("in row major index:", x_cell + (y_cell)*4000)
    return [x_cell,y_cell]

def pixel2point(point1):
    indices= np.unravel_index([x,y], prob1.shape)
    x_world= 
# def divide_map(point1):
#     # selecting random points from map enough to cover each region\
#     # We still need some sort of map division or 
#     global areas, img, publish_now, map_div_ind
#     ind=point2pixel(point1)
#     print("received indices of clicked point: ", ind)
#     areas=np.vstack((areas,ind))
#     print("areas now is: ", areas)
#     #point2pixel(point1.point.x, point1.point.y)
#     if len(areas[:,0])==4:
#         poly = np.array([areas], dtype=np.int32)
#         # if img.shape[0]!=map_.shape[0]:
#         img = np.zeros((map_.shape[0], map_.shape[1]), np.int8)
#         print("Map shapes are: ", map_.shape[0], map_.shape[1])
#         print("img shapes are:  ", img.shape)
#         img= cv2.fillPoly(img, poly, 100)
#         # img[2000:3999, 2000:3999]= 100
#         print("Unique values in image are", np.unique(img))
#         publish_now=True
#         areas= np.empty((0,2), int)
#         print("Shape of map_div_ind is: ", map_div_ind.shape)
#         map_div_ind= np.column_stack((map_div_ind,[np.argwhere(img==100)]))

        # img2=img.astype('uint8')
        # image2 = imutils.resize(img2, width=1000)
        # # cv2.imshow('image1', image1)
        # cv2.imshow('image2', image2)
        # cv2.waitKey()
        # cv2.imshow('',img)
        # cv2.waitKey(0)
        
     
def callback(data):
    if data.objects:
        print(data.objects)
    


if __name__ == '__main__':
    rospy.init_node('getPedestal', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, occupancygrid_to_numpy)
    # rospy.Subscriber("/pedestrian/object", HSE_Objects, callback)
    rospy.Subscriber("/clicked_point", PointStamped, pixel2point)
    pub =rospy.Publisher("boxes", OccupancyGrid, queue_size=1000)
    
    # pub.publish(grid)
    # rospy.Publisher('Initial_goal', PoseStamped, queue_size=10)
    # pub =rospy.Publisher("prob_goal_index", Int32MultiArray, queue_size=10)
    # # pub2=rospy.Publisher("new_prob_map",numpy_msg(Floats),queue_size=1000)
    while not rospy.is_shutdown():
        if publish_now:
            grid=numpy_to_occupancy_grid(img)
            grid1= np.array(grid.data)
            print("shape of output image is:", grid1.shape)
            unq, counts= np.unique(grid1, return_counts=True)
            print("distinct values in output image are", unq, counts)
            pub.publish(grid)
    #         # prob2= np.reshape(prob1, 16000000)
    #         # pub2.publish(prob1)
    #         setItFalse()
    rospy.spin()