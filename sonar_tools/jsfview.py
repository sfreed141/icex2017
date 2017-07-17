#! /usr/bin/python

import sys, os
import numpy as np
from scipy.signal import medfilt
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from mpl_toolkits.mplot3d import Axes3D
from pyproj import Proj
import cv2
from PIL import Image
from jsf_reader import read_jsf
from jsf_types import *

#should use  weighting factor
def plot_single_echo_return(sonar_msg, sonar_data):
    sonar_d = sonar_data
    # https://stackoverflow.com/questions/13728392/moving-average-or-running-mean/27681394#27681394
    N = 10
    cumsum = np.cumsum(np.insert(sonar_d, 0, 0))
    smoothed_data = (cumsum[N:] - cumsum[:-N]) / N

    plt.figure()

    plt.subplot(211)
    plt.plot(sonar_d, 'r', linewidth=0.5)

    plt.title('Echo returns vs sample number (raw)')
    plt.xlabel('Sample')
    plt.ylabel('Echo Return Strength')

    plt.subplot(212)
    plt.plot(smoothed_data, 'r', linewidth=0.5)

    plt.title('Echo returns vs sample number (smoothed)')
    plt.xlabel('Sample')
    plt.ylabel('Echo Return Strength')

    plt.show()

def image_from_sonar(sonar_data):
    print(len(sonar_data))
    print(len(sonar_data[0]))
    scanlines = []
    for port_data, starboard_data in zip(sonar_data[::2], sonar_data[1::2]):
        line = np.concatenate((np.flip(port_data, 0), starboard_data))
        scanlines.append(line)
    scanlines = np.array(scanlines)
    length = len(scanlines[0])
    print(length)
    for i, line in enumerate(scanlines):
        if len(line) != length:
            print('bad', i, len(line))

    #scanlines = np.array([np.concatenate((np.flip(port_data, 0), starboard_data)) for port_data, starboard_data in zip(sonar_data[::2], sonar_data[1::2])])
    #scanlines = scanlines / np.amax(scanlines)
    #scanlines = np.clip(scanlines, 0, 1)
    #scanlines *= 255
    #scanlines = np.empty((len(sonar_data) // 2, len(sonar_data[0]) * 2), dtype=np.uint16)
    #for scanline, port_data, starboard_data in zip(scanlines, sonar_data[::2], sonar_data[1::2]):
        #scanline = np.concatenate([port_data, starboard_data])
        #port_data = np.flip(port_data, 1)
        #scanlines.append(np.concatenate([port_data, starboard_data]))
        #scanlines.append(np.stack([port_data, starboard_data]))
 
    #cv2.imwrite('image.png', cv2.resize(scanlines, (0, 0), fx=0.2, fy=0.2))
    #cv2.waitKey()
    img = Image.fromarray(scanlines, 'L')
    img.show()

def process_sonar_data(jsf_msgs):
    sonar_msgs = []
    sonar_data = []

    longs = []
    lats = []
    depths = []
    for msg_hdr, msg in jsf_msgs:
        if msg_hdr['message_type'] == MESSAGE_TYPE['sonar']:
            sonar_msg, sonar_d = msg
            sonar_msgs.append(sonar_msg)
            sonar_data.append(sonar_d)

            msg = sonar_msg
            flag = msg['validity']
            if flag & 0x0001:
                # in 10000 * (minutes of arc)
                lat, lon = msg['latitude'], msg['longitude']
                lat /= 600000
                lon /= 600000

                longs.append(lon)
                lats.append(lat)
#                longlat = (lon, lat)
#                proj = Proj(init='epsg:32633')
#                proj_longlat = proj(*longlat)
#                longs.append(proj_longlat[0])
#                lats.append(proj_longlat[1])
            if flag & 0x0002:
                # units of 1/100 degree
                course = msg['compass_heading']
            if flag & 0x0004:
                # in tenths of a knot, additional precision in lsb2
                speed = msg['speed']
            if flag & 0x0020:
                # positive value indicates bow up, port up
                pitch, roll = msg['pitch'], msg['roll']
                pitch = (pitch * 180) / 32768
                roll = (roll * 180) / 32768
            if flag & 0x0040:
                # in millimeters
                altitude = msg['altitude']
            if flag & 0x0200:
                # in millimeters
                depth = msg['depth']
                depths.append(-depth / 1000)

    #plot_single_echo_return(sonar_msgs[0], sonar_data[0])
    #image_from_sonar(sonar_data)

    print(len(longs), len(lats), len(depths))

    plt.figure()
    plt.subplot(111, projection='3d')
    plt.axis('equal')
    plt.ticklabel_format(useOffset=False)
    plt.title('AUV Path')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.gca().set_zlabel('Depth')
    #plt.gca().set_zlim(-10, 0)
    plt.plot(longs, lats, depths, 'bo', markersize=0.2)
    plt.show()


def rotation_x(angle):
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rotation_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rotation_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

def process_msgs(jsf_msgs):
    # State variables to be updated from jsf messages
    sound_velocity = 1532
    depth = 0

    altitude = 0

    heave = 0
    roll = 0
    pitch = 0

    latitude = 0
    longitude = 0
    speed = 0
    heading = 0

    lastTime = 0
    lastNanoseconds = 0
    deltaTime = 0
    deltaNanoseconds = 0

    xs = []
    ys = []
    zs = []

    y = 0
    for msg_hdr, msg in jsf_msgs:
        # Update state variables
        if msg_hdr['message_type'] == MESSAGE_TYPE['bathy_attitude']:
            if msg['data_valid_flag'] & 0x01:
                heading = msg['heading']
            if msg['data_valid_flag'] & 0x02:
                heave = msg['heave']
            if msg['data_valid_flag'] & 0x04:
                pitch = msg['pitch']
            if msg['data_valid_flag'] & 0x08:
                roll = msg['roll']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_pressure']:
            if msg['data_valid_flag'] & 0x10:
                sound_velocity = msg['sound_velocity']
            if msg['data_valid_flag'] & 0x20:
                depth = msg['depth']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_altitude']:
            if msg['data_valid_flag'] & 0x1:
                altitude = msg['altitude']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_position']:
            if msg['data_valid_flag'] & 0x08:
                latitude = msg['latitude']
            if msg['data_valid_flag'] & 0x10:
                longitude = msg['longitude']
            if msg['data_valid_flag'] & 0x20:
                speed = msg['speed']
            if msg['data_valid_flag'] & 0x40:
                heading = msg['heading']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_status']:
            pass
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathymetric_data']:
            msg, data = msg

            if lastTime == 0:
                lastTime = msg['time']
                lastNanoseconds = msg['nanoseconds']
            else:
                currentTime = msg['time']
                currentNanoseconds = msg['nanoseconds']
                deltaTime = currentTime - lastTime
                if currentNanoseconds < lastNanoseconds:
                    deltaNanoseconds = lastNanoseconds - currentNanoseconds
                else:
                    deltaNanoseconds = currentNanoseconds - lastNanoseconds
                lastTime = currentTime
                lastNanoseconds = currentNanoseconds

            speed_mps = speed / 3600 * 1852
            y += deltaTime * speed_mps

            snr = data['snr_and_quality'] >> 3
            quality = data['snr_and_quality'] & 0x7
            mask = np.all([~(data['flag'] & 0x20), (snr > 20), (quality > 0)], axis=0)
            filtered_data = data[mask]

            echo_time = msg['offset_to_first_sample'] * 1e-9 + filtered_data['time_delay'] * msg['time_scale_factor']
            slant_range = sound_velocity * echo_time / 2
            angle_from_nadir = (-1)**(msg['channel'] + 1) * filtered_data['angle'] * msg['angle_scale_factor']
            seafloor_x = slant_range * np.sin(np.radians(angle_from_nadir - roll))
            seafloor_z = slant_range * np.cos(np.radians(angle_from_nadir - roll))

            for x, z in zip(seafloor_x, seafloor_z):
                xs.append(x)
                ys.append(y)
                zs.append(-z)

    plt.subplot(111, projection='3d')
    plt.plot(xs, ys, zs, 'bo', markersize=0.1)
    plt.title('Bathymetry Data')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("usage: " + sys.argv[0] + " <jsf file(s)>")
        sys.exit(0)

    jsf_msgs = []
    for jsf_filename in sys.argv[1:]:
        if not os.path.isfile(jsf_filename):
            print("error: file '" + jsf_filename + "' does not exist")
            sys.exit(0)

        jsf_msgs += read_jsf(jsf_filename)

    #process_msgs(jsf_msgs)
    process_sonar_data(jsf_msgs)

if __name__ == '__main__':
    main()
