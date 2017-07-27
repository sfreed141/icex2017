import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.basemap import Basemap
from osgeo import gdal
from pyproj import Proj

# Read the data and metadata
ds = gdal.Open(r'/home/sam/icex2017/resources/maps/malta_big.tiff')
gt = ds.GetGeoTransform()
data = ds.ReadAsArray()

xres = gt[1]
yres = gt[5]

# get the edge coordinates and add half the resolution 
# to go to center coordinates
xmin = gt[0] + xres * 0.5
xmax = gt[0] + (xres * ds.RasterXSize) - xres * 0.5
ymin = gt[3] + (yres * ds.RasterYSize) + yres * 0.5
ymax = gt[3] - yres * 0.5

ds = None

x = np.linspace(xmin, xmax, data.shape[2])
y = np.linspace(ymax, ymin, data.shape[1])
xx, yy = np.meshgrid(x, y)

# Unproject coordinates (to long/lat)
proj = Proj(init='epsg:3857')
pxmin, pymin = proj(xmin, ymin, inverse=True)
pxmax, pymax = proj(xmax, ymax, inverse=True)
pxx, pyy = proj(xx, yy, inverse=True)

extent = [pxmin, pxmax, pymin, pymax]

# Create the figure and basemap object
#fig = plt.figure()
#m = Basemap(projection='cyl', llcrnrlon=pxmin, llcrnrlat=pymin, urcrnrlon=pxmax, urcrnrlat=pymax)

# plot image
#m.pcolormesh(pxx, pyy, data[0], cmap=plt.cm.jet)

#plt.show()

fig = plt.figure()
ax = fig.gca(projection='3d')
bm = Basemap(llcrnrlon=extent[0], llcrnrlat=extent[2],
             urcrnrlon=extent[1], urcrnrlat=extent[3],
             projection='cyl', resolution='l', fix_aspect=False, ax=ax)
ax.add_collection3d(bm.drawcoastlines(linewidth=0.25))
ax.add_collection3d(bm.drawcountries(linewidth=0.35))
ax.view_init(azim = 230, elev = 50)
ax.set_xlabel(u'Longitude (°E)', labelpad=10)
ax.set_ylabel(u'Latitude (°N)', labelpad=10)
ax.set_zlabel(u'Altitude (ft)', labelpad=20)
# Add meridian and parallel gridlines
#lon_step = 1
#lat_step = 1
#meridians = np.arange(extent[0], extent[1]+lon_step, lon_step)
#parallels = np.arange(extent[2], extent[3]+lat_step, lat_step)
meridians = np.linspace(extent[0], extent[1], 10)
parallels = np.arange(extent[2], extent[3], 10)
ax.set_yticks(parallels)
ax.set_yticklabels(parallels)
ax.set_xticks(meridians)
ax.set_xticklabels(meridians)
ax.set_zlim(0., 40000.)

bm.contourf(pxx, pyy, data[0], cmap=plt.cm.Greys)

plt.show()
