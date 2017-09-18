#! /usr/bin/python

import xml.etree.ElementTree
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from jsf_reader import read_log
from pyproj import Proj
import cv2

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

proj = Proj(init='epsg:32633')
proj_longs, proj_lats = proj(log_data['longitude'], log_data['latitude'])
log_points = np.stack([proj_longs, proj_lats, -log_data['dfs_depth']], axis=1)
target_longlat = np.array(proj(14.5031, 35.9243))
longlats = np.stack([proj_longs, proj_lats], axis=1)
dist_from_target = np.linalg.norm(longlats - target_longlat, axis=1)
mask = (log_data['dfs_depth'] > 20) & (dist_from_target < 15)
log_points = log_points[mask]

cam_points = np.random.rand(10, 3)
log_points = np.random.rand(11, 3)

print(cam_points.shape, len(cam_points), cam_points.dtype)
print(log_points.shape, len(log_points), log_points.dtype)

retval, out, inliers = cv2.estimateAffine3D(cam_points, log_points)

print(retval, out, inliers)

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# plt.axis('equal')
# plt.ticklabel_format(useOffset=False)
# # ax.plot(cam_points[:,0], cam_points[:,1], cam_points[:,2], 'bo', markersize=0.5)
# ax.plot(log_points[:,0], log_points[:,1], log_points[:,2], 'go', markersize=0.5)
# ax.plot(result[:,0], result[:,1], result[:,2], 'ro', markersize=0.5)

# plt.show()