#! /usr/bin/python

import xml.etree.ElementTree
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

transforms = []
e = xml.etree.ElementTree.parse('/home/sam/Desktop/cam.xml').getroot()
for cam in e.findall('chunk/cameras/camera'):
    #print(cam.get('id'), cam.get('label'))
    if cam.get('enabled') == 'true':
        transform = np.fromstring(cam[0].text, sep=' ').reshape((4,4))
        print(transform.dtype, transform)
        transforms.append(transform)

points = []
origin = np.array([0., 0, 0, 1])
for transform in transforms:
    point = transform.dot(origin)
    points.append(point)
points = np.array(points)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], 'bo')
plt.show()
