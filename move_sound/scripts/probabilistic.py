#!/usr/bin/env python3

"""Probabilistic.ipynb
Automatically generated by Colaboratory.
Original file is located at
    https://colab.research.google.com/drive/1dQjpj4NVTvmcUHT85zqHaR38IiRkqLzZ
"""
import rospy
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int32MultiArray
# import math
# from numpy import random
# import matplotlib.pyplot as plt
# import seaborn as sns
import numpy as np
# from collections import Counter


# arr1= random.normal(loc=5, scale= 5 , size=360)
# arr1= np.round(arr1)
# print(arr1)
# sns.distplot(arr1, hist=False)
# # plt.show()
# print(arr1[np.argmax(arr1)])

# arr1= arr1.astype(int)
# ind_arr= Counter(arr1)
# prob= np.zeros(360,dtype='int32') # Should I convert zeros to 0.00001 ?
# # prob= prob.astype(int)
# for ele in ind_arr:    
#   prob[ele]= ind_arr[ele]

# prob= prob/len(prob)
# prob_sum= np.sum(prob)
# print(prob_sum)
# arr2= random.normal(loc=10, scale= 5 , size=360)
# arr2= np.round(arr2)
# arr2= arr2.astype(int)

# ind_arr2= Counter(arr2)
# prob2= np.zeros(360, dtype='int32')
# prob2= prob2.astype(int)

# for ele in ind_arr2:    
#   prob2[ele]= ind_arr2[ele]

# prob2= prob2/len(prob2)
# print("prob2 is: ",prob2)
# prob= np.transpose(prob)
# print("transpose is", prob)
# prob2= prob*prob2
# print("prob2 after multiplication: ", prob2)

# # Normalize
# prob2= prob2/prob2.sum()
# prob2_sum= np.sum(prob2)
# print(prob2_sum)
# def callback(data):
#   print("Subscribed messages are: ", data.data)
#   print(data.data[0])
  # time_now= rospy.Time.now()
  # rospy.sleep(1)
  # time_now2= rospy.Time.now()
  # if time_now2>time_now:
  #   print("yes")
  # else:
  #   print("no")
  # dim1=data.data[4]
  # dim2=data.data[5]
  # prob1=np.ones(dim1,dim2)
  # prob2=np.zeros(dim1,dim2)
   
  
# This section is for map division
# def map_division(inp_arr):
#   current_div= len(inp_arr)
#   new_div= np.array(8)
#   our_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
#   chunk_size = 3
#   chunked_list = [our_list[i:i+chunk_size] for i in range(0, len(our_list), chunk_size)]
#   print(chunked_list)

# if __name__ == '__main__':
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber("/mean_index_cell", Int32MultiArray, callback)
#     rospy.spin()


#######################

ind= []
arr= np.zeros((8,8))
arr[2:6, 2:6]=1
print(np.argwhere(arr==1.0))
print(np.argwhere(arr==1.0).shape)
# print("shape of ind array", ind.shape)
new_ind= np.argwhere(arr==1).tolist()
ind.append(new_ind)
# print(ind)
new_ind= np.argwhere(arr==0).tolist()
print(new_ind)
ind.append(ind)
print(ind[1])
# ind[0,:,:]=[np.argwhere(arr==1.0)]
# ind=np.column_stack((ind, [np.argwhere(arr==1.0)]))
# print("Before readdition",ind.shape)
# ind= np.column_stack((ind, [np.argwhere(arr==0.0)]))
# print("after readdition",ind.shape)
