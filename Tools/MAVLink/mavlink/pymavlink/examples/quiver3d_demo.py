import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax = Axes3D(fig)
                     
# x, y = np.meshgrid(np.arange(-0.8, 1),
#                       np.arange(-0.8, 1))
x, y, z = np.meshgrid(np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.8))

# u = np.sin(np.pi * x) * np.cos(np.pi * y)
# v = -np.cos(np.pi * x) * np.sin(np.pi * y)
u = np.sin(np.pi * x) * np.cos(np.pi * y) * np.cos(np.pi * z)
v = -np.cos(np.pi * x) * np.sin(np.pi * y) * np.cos(np.pi * z)
w = (np.sqrt(2.0 / 3.0) * np.cos(np.pi * x) * np.cos(np.pi * y) *
     np.sin(np.pi * z))

ax.quiver(x, y, z, u, v, w, units='dots', length=0.1) #scale_units='x', scale=1)
#x.plot(u, v)

plt.show()

