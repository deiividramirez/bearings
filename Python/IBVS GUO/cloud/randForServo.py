"""
	Tuesday April 17th, 01:35:17 2018
	@author: robotics lab (Patricia Tavares)
	@email: patricia.tavares@cimat.mx
	version: 1.0
	This code is used to obtain a random point cloud for 
	computing the essential matrix. This is only use for the 
	script EpipolarVisualServo
"""

import numpy as np
from random import random

#the amount of points
n = 20

#to store the coords
x = []
y = []
z = []

#the limits in the axes
xlim_u = 1.5#upper
xlim_l = 0.5#lower
ylim_u = 4.0#upper
ylim_l = 3.0#lower
zlim_u = 0.3#upper
zlim_l = -0.3#lower

for i in range(n):
	x.append(xlim_l + random()*(xlim_u-xlim_l))
	y.append(ylim_l + random()*(ylim_u-ylim_l))
	z.append(zlim_l + random()*(zlim_u-zlim_l))

x = np.array(x)
y = np.array(y)
z = np.array(z)

#save the data in files
np.savetxt('x.data',x)
np.savetxt('y.data',y)
np.savetxt('z.data',z)
	
