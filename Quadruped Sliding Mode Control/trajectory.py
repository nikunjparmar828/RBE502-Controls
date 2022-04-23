# Trajectory testing and ploting 

# importing mplot3d toolkits, numpy and matplotlib
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

def trajectory(t):

	x = (-1/42525000)* (t^5) + (7/1215000) * (t^4) + (-251/567000)*(t^3) + (113/9720)*(t^2) + (-325/6804)*(t)

	y = (2/26578125)* (t^5) + (-7/607500) * (t^4) + (79/141750)*(t^3) + (-211/24300)*(t^2) + (1313/42525)*(t)

	z = (1/11375000)* (t^5) + (-1/65000) * (t^4) + (89/91000)*(t^3) + (-71/2600)*(t^2) + (2857/9100)*(t)

	return (x,y,z)



fig = plt.figure()

# syntax for 3-D projection
ax = plt.axes(projection ='3d')

# defining all 3 axes
x,y,z = trajectory(5)

# plotting
ax.plot3D(x, y, z, 'green')
ax.set_title('3D line plot geeks for geeks')
plt.show()
