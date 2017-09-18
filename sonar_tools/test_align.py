#! /usr/bin/python

import xml.etree.ElementTree
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from jsf_reader import read_log
import pcl
from pcl.registration import icp
from pyproj import Proj

def rigid_transform_3D(A, B):
    assert(len(A) == len(B))

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.transpose(AA) * BB

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if np.linalg.det(R) < 0:
       print("Reflection detected")
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    print(t)

    return R, t

def get_camera_points(filename):
    transforms = []
    e = xml.etree.ElementTree.parse(filename).getroot()
    for cam in e.findall('chunk/cameras/camera'):
        #print(cam.get('id'), cam.get('label'))
        if cam.get('enabled') == 'true':
            transform = np.fromstring(cam[0].text, sep=' ').reshape((4,4))
            #print(transform.dtype, transform)
            transforms.append(transform)

    points = []
    origin = np.array([0., 0, 0, 1])
    for transform in transforms:
        point = transform.dot(origin)
        points.append(point)
    points = np.array(points)

    return points[:, 0:3]

def register_icp(cam_points, log_points):
    cam_pc = pcl.PointCloud(cam_points.astype(np.float32))
    log_pc = pcl.PointCloud(log_points.astype(np.float32))

    converged, transform, estimate, fitness = None, None, None, float('inf')
    for scale in np.arange(0.1, 1, 0.1):
        scaled_cam_pc = pcl.PointCloud((scale * cam_points).astype(np.float32))
        c, t, e, f = icp(scaled_cam_pc, log_pc, max_iter=100000)
        if f < fitness:
            print(scale)
            converged, transform, estimate, fitness = c, t, e, f

    print(converged, transform, estimate, fitness)
    result = estimate.to_array()

    fig = plt.figure()
    
    ax = fig.add_subplot(121, projection='3d')
    plt.axis('equal')
    plt.ticklabel_format(useOffset=False)
    ax.plot(cam_points[:,0], cam_points[:,1], cam_points[:,2], 'bo', markersize=0.5)
    
    ax = fig.add_subplot(122, projection='3d')
    plt.axis('equal')
    plt.ticklabel_format(useOffset=False)
    ax.plot(log_points[:,0], log_points[:,1], log_points[:,2], 'go', markersize=0.5)
    ax.plot(result[:,0], result[:,1], result[:,2], 'ro', markersize=0.5)

    plt.show()

# modified from python-pcl/random_sample_consensus.py
def register_ransac(cam_points, log_points):
    cam_pc = pcl.PointCloud(cam_points.astype(np.float32))
    log_pc = pcl.PointCloud(log_points.astype(np.float32))

    model = pcl.SampleConsensusModelRegistration(cam_pc)
    ransac = pcl.RandomSampleConsensus(model)
    ransac.set_DistanceThreshold(0.01)
    ransac.computeModel()
    inliers = ransac.get_Inliers()

    if len(inliers) != 0:
        finalpoints = np.zeros((len(inliers), 3), dtype=np.float32)

        for i in range(0, len(inliers)):
            finalpoints[i][0] = cloud[inliers[i]][0]
            finalpoints[i][1] = cloud[inliers[i]][1]
            finalpoints[i][2] = cloud[inliers[i]][2]
        
        final.from_array(finalpoints)
    
def view_data(data_a, data_b):
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    plt.axis('equal')
    plt.ticklabel_format(useOffset=False)
    ax.plot(data_a[:,0], data_a[:,1], data_a[:,2], 'bo', markersize=0.5)
    ax = fig.add_subplot(122, projection='3d')
    plt.axis('equal')
    plt.ticklabel_format(useOffset=False)
    ax.plot(data_b[:,0], data_b[:,1], data_b[:,2], 'ro', markersize=0.5)
    plt.show()

# cam_pc.to_file(b"cam_pc.pcd")
# log_pc.to_file(b"log_pc.pcd")

# transformed_cam_pc = pcl.PointCloud()
# transformed_cam_pc.from_file(b"ransac/build/output.pcd")
# result = transformed_cam_pc.to_array()

def main():
    # Location of camera alignment data (from Photoscan)
    cam_filename = '/home/sam/Desktop/cam.xml'
    # Location of corrected log file
    log_filename = '/home/sam/Downloads/jun29_plane_beaufighter/logs/20100101_080859_UTC_0_beaufighter_revisit_IVER3-3013_Corr.log'

    # Parse files
    cam_points = get_camera_points(cam_filename)
    log_data = read_log(log_filename)

    # Filter log data
    proj = Proj(init='epsg:32633')
    proj_longs, proj_lats = proj(log_data['longitude'], log_data['latitude'])
    log_points = np.stack([proj_longs, proj_lats, -log_data['dfs_depth']], axis=1)

    target_longlat = np.array(proj(14.5031, 35.9243))
    longlats = np.stack([proj_longs, proj_lats], axis=1)
    dist_from_target = np.linalg.norm(longlats - target_longlat, axis=1)
    mask = (log_data['dfs_depth'] > 20) & (dist_from_target < 15)
    log_points = log_points[mask]

    # log_points = log_points[np.sort(np.random.choice(len(log_points), len(cam_points), False))]

    # print(cam_points.shape, len(cam_points), cam_points.dtype)
    # print(log_points.shape, len(log_points), log_points.dtype)

    # Compute centroids and offset data
    cam_centroid, log_centroid = np.mean(cam_points, axis=0), np.mean(log_points, axis=0)
    cam_points = cam_points - cam_centroid
    log_points = log_points - log_centroid

    # Display the two point clouds side by side
    # view_data(cam_points, log_points)

    register_icp(cam_points, log_points)


if __name__ == '__main__':
    main()
