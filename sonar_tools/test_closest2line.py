#! /usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

# Based on http://www.fundza.com/vectors/point2line/index.html
def closest_point_to_line(point, line_start, line_end):
    line_vec = line_end - line_start
    point_vec = point - line_start
    
    point_vec = point_vec / np.linalg.norm(line_vec)
    line_unit_vec = line_vec / np.linalg.norm(line_vec)

    t = np.dot(point_vec, line_unit_vec)
    t = np.clip(t, 0.0, 1.0)

    closest_point = line_start + t * line_vec
    distance = np.linalg.norm(point - closest_point)

    return (closest_point, distance)

# Vectorized version of above
def closest_point_to_path(point, path):
    line_start, line_end = path[:-1], path[1:]
    line_vecs = line_end - line_start
    point_vecs = point - line_start

    point_vecs = point_vecs / np.linalg.norm(line_vecs, axis=1)[:, np.newaxis]
    line_unit_vecs = line_vecs / np.linalg.norm(line_vecs, axis=1)[:, np.newaxis]

    t = np.inner(point_vecs, line_unit_vecs).diagonal()
    t = np.clip(t, 0.0, 1.0)

    closest_points = line_start + t[:, np.newaxis] * line_vecs
    distance = np.linalg.norm(point - closest_points, axis=1)

    return (closest_points, distance)

path = np.array([
    [0, 0],
    [1, 0],
    [1.5, .5],
    [2, 1],
    [3, 2],
    [4, 1]
])

point = np.array([.5, 1])

dists = cdist([point], path)
print(dists)
print(dists.argmin())

closest = []
dist = []
if True:
    closest, dist = closest_point_to_path(point, path)
else:
    for line_start, line_end in zip(path[0::], path[1::]):
        c, d= closest_point_to_line(point, line_start, line_end)
        closest.append(c)
        dist.append(d)
    closest = np.array(closest)
print(closest)
print(dist)

plt.axis('equal')
plt.plot(path[:, 0], path[:, 1], 'b')
plt.plot(path[:, 0], path[:, 1], 'bo')
plt.plot(point[0], point[1], 'ro')
plt.plot(closest[:,0], closest[:,1], 'go')
plt.show()
