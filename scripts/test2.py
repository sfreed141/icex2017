#! /usr/bin/python

from osgeo import gdal
from osgeo import osr
from osgeo.gdalconst import *

#malta_ds = gdal.Open("malta_close.tiff")
malta_ds = gdal.Open("test.png")

driver = gdal.GetDriverByName( 'GTiff' )
ds = driver.CreateCopy( 'out_file.tiff', malta_ds)

proj = osr.SpatialReference()
proj.SetWellKnownGeogCS( "EPSG:4326" )
ds.SetProjection(proj.ExportToWkt())
geotransform = (1,0.1,0.5,40,0.2,0.1)
ds.SetGeoTransform(geotransform)
ds = None

