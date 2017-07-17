#! /usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from osgeo import gdal
from pyproj import Proj

# Modified from https://gis.stackexchange.com/questions/184727/plotting-raster-maps-in-python
gdata = gdal.Open("../Maps/malta_close.tiff")
gt = gdata.GetGeoTransform()
data = gdata.ReadAsArray()

xres = gt[1]
yres = gt[5]

xmin = gt[0] + xres * 0.5
xmax = gt[0] + (xres * gdata.RasterXSize) - xres * 0.5
ymin = gt[3] + (yres * gdata.RasterYSize) + yres * 0.5
ymax = gt[3] - yres * 0.5

x_center = (xmin + xmax) / 2
y_center = (ymin + ymax) / 2

print("gt: ", gt)

proj = Proj(init='epsg:3857')
xmin, ymin = proj(xmin, ymin, inverse=True)
xmax, ymax = proj(xmax, ymax, inverse=True)
x_center, y_center = proj(x_center, y_center, inverse=True)

print("xmin: ", xmin)
print("xmax: ", xmax)
print("ymin: ", ymin)
print("ymax: ", ymax)
print("x_center: ", x_center)
print("y_center; ", y_center)

#m = Basemap(epsg=3857, llcrnrlon=xmin, llcrnrlat=ymin, urcrnrlon=xmax, urcrnrlat=ymax)
m = Basemap(epsg=3857, width=10000, height=10000, lon_0=14.502779582300805, lat_0=35.90033295339112)

m.drawmapboundary(fill_color='aqua')
m.fillcontinents(color='coral',lake_color='aqua')

#parallels = np.arange(10., 20., 0.25)
#meridians = np.arange(30., 40., 0.25)
#m.drawparallels(parallels, labels=[0,1,1,0],fontsize=12,linewidth=0.4)
#m.drawmeridians(meridians, labels=[1,0,0,1],fontsize=12,linewidth=0.4)

#x = np.linspace(0, m.urcrnrx, data.shape[1])
#y = np.linspace(0, m.urcrnry, data.shape[0])

#xx, yy = np.meshgrid(x, y)

#im = m.pcolormesh(xx, yy, data.T, cmap=plt.cm.jet)
#im = m.imshow(xx, yy, data)

plt.show()
#plt.savefig('test.png')
