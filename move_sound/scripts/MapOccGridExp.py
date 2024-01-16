#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import itertools
from collections import Counter
import pandas as pd
from sklearn import preprocessing
import rospy
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import tf_conversions
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import ros_numpy
from nav_msgs.msg import OccupancyGrid
# from occupancy_grid_python import OccupancyGridManager
# def distmat_v2(a, index):
#     i,j = np.indices(a.shape, sparse=True)
#     return np.sqrt((i-index[0])**2 + (j-index[1])**2)
cur_val=[]
goal_index= Int32()
map1=np.ones(10)
publish_sig=False

def checkGoal(cur):
    global cur_val, goal_index,map1, publish_sig
    cur_val=cur.data
    dim1,dim2= map1.shape
    print("Map Value at cur_index:", map1[cur_val[0], cur_val[1]])
    if map1[cur_val[0], cur_val[1]]!=0:
        indices= np.argwhere(map1==0)
        # print("Indices are:", indices)
        dist= np.sqrt((indices[:,0]-cur_val[0])**2  + (indices[:,1]-cur_val[1])**2)
        dist= np.column_stack((indices, dist))
        dist = dist[dist[:, 2].argsort()] # sort the indices wrt dist from curr goal
        print("dist[0] is ,:   ", dist[0])
        
        for i,j,k in dist:
            i=int(i)
            j=int(j)
            if(map1[i,j]==0 and (i!=cur_val[0] or j!=cur_val[1])):
                goal_index.data= i+ j*dim2
                break
        print("Goal_index after considering obstacles is: ", goal_index)
    else:
        goal_index.data= cur_val[0] + dim2*cur_val[1]
        print("Goal is already in free map pixel", goal_index)
    publish_sig=True

def callback(map):
    global map1
    map1=np.array(map.data)
    map1=map1.reshape(4000,4000).T
    # since map is not rectangular but equal dimensions reshape.T method works fine
    print("Unique values are: ",np.unique(map1))
    print("Total unoccupied cells", np.count_nonzero(map1 == 0))
    print("Total occupied cells", np.count_nonzero(map1 == 100))
    print("Total unknown cells", np.count_nonzero(map1 == -1))

    # Lets suppose current val of goal is: 50,50
    # cur_val=[50,50]
    
    # num_list=[range(cur_val[0]-50,cur_val[0]+50)]
    # ind0_list= [item for item in num_list if item >= 0]
    # num_list2=[range(cur_val[1]-50,cur_val[1]+50)]
    # ind1_list= [item for item in num_list2 if item >= 0]
    # #Lets slice 100,100 matrix with cur_val at center
    # map_50= map1[min(ind0_list)]
    # dist_map=distmat_v2(map1, cur_val)
    # for i in range(cur_val[0]-25, cur_val[0]+25):
    #     for j in range(cur_val[1]-25 , cur_val[1]+25):

    # print(map1[0])
    # open_ind= np.where(map1<0.196)
    # print(open_ind)
    # obs_ind= np.where(map1>0.65, dtype=float)
    # print(obs_ind)
    

if __name__ == '__main__':
    rospy.init_node('map_occ_grid_node', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.Subscriber("/prob_goal_index", Int32MultiArray, checkGoal)
    pub=rospy.Publisher("/goal_after_check", Int32, queue_size=10)
    while not rospy.is_shutdown():
        if publish_sig:
            pub.publish(goal_index)
            publish_sig=False
    rospy.spin()
