#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from mpl_toolkits.mplot3d import Axes3D


# class Visualiser:
#     def __init__(self):
#         self.fig, self.ax = plt.subplots()
#         self.ln, = plt.plot([], [], 'ro')
#         self.x_data, self.y_data = [] , []

#     def plot_init(self):
#         self.ax.set_xlim(0, )
#         self.ax.set_ylim(-7, 7)
#         return self.ln
    
#     def getYaw(self, pose):
#         quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
#                 pose.orientation.w)
#         euler = tf.transformations.euler_from_quaternion(quaternion)
#         yaw = euler[2] 
#         return yaw   

def odom_callback(data):
    plt.close('all')
    map=data.data
    print("shape of map is", map.shape)
    map= np.reshape(map, [4000,4000])
    x = range(len(map))
    y = range(len(map))
    hf = plt.figure()
    ha = hf.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(x, y)  # `plot_surface` expects `x` and `y` data to be 2D
    ha.plot_surface(X, Y, map)
    print("max prob value", np.amax(map))
    print("Min prob value is: ",np.amin(map))
    # print("SOme values near curr_cell", map1[])
    plt.show()  


rospy.init_node('lidar_visual_node')
# vis = Visualiser()
sub = rospy.Subscriber('new_prob_map', numpy_msg(Floats), odom_callback)
rospy.spin()
# ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
# plt.show() 