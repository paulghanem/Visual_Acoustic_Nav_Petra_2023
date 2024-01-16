import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import multivariate_normal
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import math
from collections import Counter
mean1=200
mean2=200
dim1=400
dim2=400
val = 50
mean = np.array([mean1,mean2])
cov = np.array([[val,1], [1, val]])
distr = multivariate_normal(cov = cov, mean = mean)#,seed = random_seed)
data = distr.rvs(size = 9000)
print("First 5 values before rounding are:  ",data[0:5,:])
data=np.round(data)
print("First 5 values after rounding are:  ",data[0:5,:])
   # We need to disregard values which are out of bounds of map
   # anything more than 3999 and less than 0 as of now and dim1,2 later
df = pd.DataFrame(data, columns = ['A','B'])
df = df[df['A'].between(0, dim1-1)]
df = df[df['B'].between(0, dim2-1)]
data= df.to_numpy()
data=np.abs(data)
print("data after removing outbounds",data)
b = np.ascontiguousarray(data).view(np.dtype((np.void, data.dtype.itemsize * data.shape[1])))
unq_data, unq_cnt = np.unique(b, return_counts=True)
unq_data = unq_data.view(data.dtype).reshape(-1, data.shape[1])
prob=np.zeros([dim1,dim2], dtype=float) # Taking ones matrix instead of zero to provide minimal prob to all pixels 
   # counter=0

unq_data=unq_data.astype(np.int)
unq_cnt=unq_cnt ##Adding one to all counts :)
print("unq_data is: ", unq_data[np.argmax(unq_cnt)])
print("unq_cnt is: ", unq_cnt[np.argmax(unq_cnt)])
prob[unq_data[:,0], unq_data[:,1]] = unq_cnt
print("mat count is: ",prob)
prob=np.divide(prob, np.sum(prob))
print("prob matrix sum is: ", np.sum(prob))
x = range(len(prob))
y = range(len(prob))
# data = numpy.random.random((nx, ny))
hf = plt.figure()
ha = hf.add_subplot(111, projection='3d')
X, Y = np.meshgrid(x, y)  # `plot_surface` expects `x` and `y` data to be 2D
ha.plot_surface(X, Y, prob)
print("Max prob value is: ",np.amax(prob))
print("Min prob value is: ",np.amin(prob))
# print(" 50 Values near point", prob[990:1010, 990:1010])
# print("Values which are not 1")
plt.show()
# plt.plot(range(prob[0,:]),range(prob[:,1]), 'o', c='lime',
#             markeredgewidth = 0.5,
#             markeredgecolor = 'black')
# plt.show()