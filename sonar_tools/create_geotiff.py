#! /usr/bin/python

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from pyproj import Proj
from osgeo import gdal, osr
import cv2

from jsfview import read_jsf

def main():
    if len(sys.argv) < 4:
        print("usage: ./create_geotiff.py <jsf_filename> <png_filename> <out_filename>")
        sys.exit(1)

    jsf_filename = sys.argv[1]
    png_filename = sys.argv[2]
    out_filename = sys.argv[3]

    sonar_msgs = read_jsf(jsf_filename)

    first_sonar = sonar_msgs[0]
    longs = []
    lats = []
    for d in sonar_msgs:
        if d['validity'] & 1 != 0:
            first_sonar = d
            lon, lat = d['longitude'], d['latitude']
            lon = lon / 600000
            lat = lat / 600000
            longs.append(lon)
            lats.append(lat)

    first_longlat = (longs[0], lats[0])
    last_longlat = (longs[-1], lats[-1])

    proj = Proj(init='epsg:32633')
    first_proj = proj(*first_longlat)
    last_proj = proj(*last_longlat)

    print("First ping (longlat): {} {}".format(*first_longlat))
    print("Last ping (longlat): {} {}".format(*last_longlat))

    print("First ping (projected): {} {}".format(*first_proj))
    print("Last ping (projected): {} {}".format(*last_proj))

    top = np.array(first_proj)
    bot = np.array(last_proj)
    unit_vector = (top - bot) / np.linalg.norm(top - bot)
    rotated_vector = 80 * np.array((-unit_vector[1], unit_vector[0]))
    left_point = top + rotated_vector
    left_point = tuple(left_point)

    cv_png = cv2.imread(png_filename, cv2.IMREAD_GRAYSCALE)
    img_height = cv_png.shape[0]
    img_width = cv_png.shape[1]

    geopoints = np.array([left_point, first_proj, last_proj], np.float32)
    rasterpoints = np.array([(0, 0), (img_width / 2, 0), (img_width / 2, img_height)], np.float32)
    af = cv2.getAffineTransform(rasterpoints, geopoints)
    print(af)
    af = af.reshape((6,))
    print(af)
    geotransform = (af[2], af[0], af[1], af[5], af[3], af[4])

    src_png = gdal.Open(png_filename)

    driver = gdal.GetDriverByName('GTiff')
    ds = driver.CreateCopy('out_file.tiff', src_png)

    proj = osr.SpatialReference()
    proj.SetWellKnownGeogCS("EPSG:32633")
    ds.SetProjection(proj.ExportToWkt())
    ds.SetGeoTransform(geotransform)
    ds = None

if __name__ == '__main__':
    main()

