#! /usr/bin/python

import xml.etree.ElementTree
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from jsf_reader import read_log
import pcl
from pcl.registration import icp

def get_camera_points(filename):
    transforms = []
    e = xml.etree.ElementTree.parse(filename).getroot()
    for cam in e.findall('chunk/cameras/camera'):
        #print(cam.get('id'), cam.get('label'))
        if cam.get('enabled') == 'true':
            transform = np.fromstring(cam[0].text, sep=' ').reshape((4,4))
            #print(transform.dtype, transform)
            transforms.append(transform)

    # TODO: vectorize
    points = []
    origin = np.array([0., 0, 0, 1])
    for transform in transforms:
        point = transform.dot(origin)
        points.append(point)
    points = np.array(points)

    return points[:, 0:3]

cam_points = get_camera_points('/home/sam/Desktop/cam.xml')
log_data = read_log('/home/sam/Downloads/jun29_plane_beaufighter/logs/20100101_080859_UTC_0_beaufighter_revisit_IVER3-3013_Corr.log')

log_points = np.stack([log_data['longitude'], log_data['latitude'], -log_data['dfs_depth']], axis=1)

print(cam_points.shape, len(cam_points), cam_points.dtype)
print(log_points.shape, len(log_points), log_points.dtype)

cam_pc = pcl.PointCloud(cam_points.astype(np.float32))
log_pc = pcl.PointCloud(log_points.astype(np.float32))

converged, transform, estimate, fitness = icp(cam_pc, log_pc, max_iter=None)

print(converged, transform, estimate, fitness)
result = estimate.to_array()

fig = plt.figure()
ax = fig.gca(projection='3d')
plt.axis('equal')
plt.ticklabel_format(useOffset=False)
ax.plot(cam_points[:,0], cam_points[:,1], cam_points[:,2], 'bo', markersize=0.5)
ax.plot(log_points[:,0], log_points[:,1], log_points[:,2], 'go', markersize=0.5)
ax.plot(result[:,0], result[:,1], result[:,2], 'ro', markersize=0.5)

plt.show()