#!/usr/bin/env python3

import numpy as np
import itertools
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#angle1= list(np.arange(0.1,180, 0.2))
#angle2= list(np.arange(0.1,180, 0.2))

#combos= np.array(np.meshgrid(angle1, angle2)).T.reshape(-1,2)
##combos= [x for x in itertools.product(angle1, angle2) if sum(x)<180]
#sum1=np.sum(combos,axis=1)
#combos= np.column_stack((combos, sum1))
#combos= combos[combos[:,2]<180]
#combos= combos[:,0:2]
#print(combos)
#angle3=180- np.sum(combos, axis=1)
##print(angle3)
#combos= np.column_stack((combos, angle3))
#sines= np.sin(np.radians(combos))
## Assuming the sensors are at 1 ft distance
## dist1= 0.3048* np.divide(sines[:,1], sines[:,2])
#dist1= 0.2286* np.divide(sines[:,1], sines[:,2])
#print(dist1)
#graph= np.column_stack((combos[:,0], combos[:,1]))
#graph= np.column_stack((graph,dist1))
## hf = plt.figure()
## ha = hf.add_subplot(111, projection='3d')
## `plot_surface` expects `x` and `y` data to be 2D
##  A = graph
## print(graph)
## X,Y,Z = graph.T # with Z the values at points x,y
## plt.scatter(X,Y,c=Z) 
## plt.colorbar()
## plt.show()

#ax = plt.axes(projection='3d')

## Data for a three-dimensional line
#zline = np.linspace(np.min(graph[:,2]), np.max(graph[:,2])+1, len(graph[:,2]))
#xline = np.linspace(0, 181, len(graph[:,0]))
#yline = np.linspace(0,181, len(graph[:,0]))
##ax.plot3D(xline, yline, zline, 'red')

## Data for three-dimensional scattered points
#zdata = graph[:,2]
#xdata = graph[:,0]
#ydata = graph[:,1] #, cmap='Greens'
#ax.scatter3D(xdata, ydata, zdata, c=zdata);
#plt.show()

#x = np.linspace(0,np.pi,len(graph[:,2]))
#y = np.linspace(0,np.pi,len(graph[:,2]))

#z = x**2 + y**2

#fig = plt.figure()

#ax = fig.add_subplot(111, projection='3d')

#ax.plot(x,y,z)

#plt.show()
x=np.arange(-50,50,1)
y=-(x)**2 +50**2 
y2= np.max(y)-y
plt.plot(x,y, label='y=-x**2 +50**2')
plt.plot(x,y2, label='ymax -y ')
plt.legend()
plt.show()
