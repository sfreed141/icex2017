#! /usr/bin/python

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from jsf_reader import read_log
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise

def plot_raw(log_data):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    plt.axis('equal')
    plt.ticklabel_format(useOffset=False)
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Depth')

    plt.plot(log_data['longitude'], log_data['latitude'], -log_data['dfs_depth'], 'ro', markersize=0.2)

    plt.show()

def plot_filtered(log_data):
    def f(x, dt):
        """ state transition function """
        F = np.array([[1, dt, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, dt],
                      [0, 0, 0, 1]])
        return np.dot(F, x)

    def h(x):
        """ measurement function (state to measurement transform) """
        return np.array(x[0], x[2])

    dt = 1.
    points = MerweScaledSigmaPoints(n=4, alpha=.1, beta=2., kappa=-1)
    ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, fx=f, hx=h, dt=dt, points=points)
    ukf.x = np.array([0., 0., 0., 0.])
    ukf.R = np.diag([0.09, 0.09])
    ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=1, var=0.02)
    ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=1, var=0.02)

    uxs = []
    zs = np.stack((log_data['longitude'], log_data['latitude']), axis=-1)
    for z in zs:
        ukf.predict()
        ukf.update(z)
        uxs.append(ukf.x.copy())
    uxs = np.array(uxs)

    plt.subplot(211)
    plt.plot(log_data['longitude'], log_data['latitude'])

    plt.subplot(212)
    plt.plot(uxs[:, 0], uxs[:, 2])

    plt.show()

def main():
    if len(sys.argv) < 2:
        print('usage: ' + sys.argv[0] + ' <log file>')
        sys.exit(0)

    log_filename = sys.argv[1]
    log_data = read_log(log_filename)
    # plot_raw(log_data)
    plot_filtered(log_data)

if __name__ == '__main__':
    main()
