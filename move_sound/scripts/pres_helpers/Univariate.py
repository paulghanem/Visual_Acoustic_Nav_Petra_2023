from numpy import random
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from collections import Counter

#Creating a normal distribution of 360 samples as I want a normal distribution of angle of sound source
arr1= random.normal(loc=60, scale= 13 , size=4000)
# Then I am rounding off those values
arr1= np.round(arr1)
arr1= arr1%360  # Circulating all values into continous angles of range(0-360)
print(arr1)

arr1= arr1.astype(int)
ind_arr= Counter(arr1)
prob= np.zeros(360,dtype='int32')
for ele in ind_arr:    
  prob[ele]= ind_arr[ele]
prob= np.divide(prob, np.sum(prob))

prob_sum= np.sum(prob)
print(prob_sum)

plt.bar(range(360), prob)
plt.show()