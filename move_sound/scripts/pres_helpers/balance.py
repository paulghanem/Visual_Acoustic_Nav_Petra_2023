import numpy as np
import matplotlib.pyplot as plt
x=np.arange(-50,50,1)
y=-(x)**2 +50**2 
y2= np.max(y)-y
plt.plot(x,y, label='y=-x**2 +50**2')
plt.plot(x,y2, label='ymax -y ')
plt.legend()
plt.show()