#! /usr/bin/python

import numpy as np
from numpy import cos, sin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pcl
from pcl.registration import icp, gicp, icp_nl

theta = [0., 0., np.pi]
rot_x = [[1,              0,              0],
         [0,              cos(theta[0]), -sin(theta[0])],
         [0,              sin(theta[0]),  cos(theta[0])]]
rot_y = [[cos(theta[1]),  0,              sin(theta[1])],
         [0,              1,              0],
         [-sin(theta[1]),  0,             cos(theta[1])]]
rot_z = [[cos(theta[2]), -sin(theta[1]),  0],
         [sin(theta[2]),  cos(theta[1]),  0],
         [0,              0,              1]]
transform = np.dot(rot_x, np.dot(rot_y, rot_z))

source = np.random.randn(100, 3)
source_pc = pcl.PointCloud(source.astype(np.float32))
target = np.dot(transform, source.T).T
target_pc = pcl.PointCloud(target.astype(np.float32))

converged, transf, estimate, fitness = icp(source_pc, target_pc, max_iter=1000)

print(converged, transf, estimate, fitness)
result = estimate.to_array()

fig = plt.figure()
ax = fig.add_subplot(221, projection='3d')
ax.scatter(source[:,0], source[:,1], source[:,2])
ax = fig.add_subplot(222, projection='3d')
ax.scatter(target[:,0], target[:,1], target[:,2])
ax = fig.add_subplot(212, projection='3d')
ax.scatter(source[:,0], source[:,1], source[:,2], 'b')
ax.scatter(result[:,0], result[:,1], result[:,2], 'r')

plt.show()
