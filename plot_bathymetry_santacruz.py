import glob
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import xarray as xr
from mpl_toolkits.basemap import Basemap
import gc

import matplotlib
matplotlib.rc('font', size=12)

data_path = 'processed_netcdf'

multibeam_files = glob.glob(data_path + '/*2021101[34]*.nc')
multibeam_files.sort()

lon0, lon1 = -122.10, -121.98
lat0, lat1 = 36.91, 36.97

parallels = np.arange(lat0, lat1 + 0.02, 0.02)
meridians = np.arange(lon0, lon1 + 0.02, 0.02)

fig = plt.figure(figsize=(12, 6))
map = Basemap(llcrnrlon=lon0, llcrnrlat=lat0, urcrnrlon=lon1, urcrnrlat=lat1, \
              resolution='f')
map.drawcoastlines()
map.drawparallels(parallels, labels=~np.isnan(parallels))
map.drawmeridians(meridians, labels=~np.isnan(meridians))

skip = 4

for f in multibeam_files:
    print('Plotting ', f)
    ds = xr.open_dataset(f)
    lon = np.array(ds.longitude[::skip,::skip])
    lat = np.array(ds.latitude[::skip,::skip])
    depth = np.array(ds.depth[::skip,::skip])
    depth[depth > 100] = np.nan

    plt.pcolor(lon, lat, depth, vmin=10, vmax=50, cmap=cm.turbo_r)
    del lon, lat, depth, ds
    gc.collect()

plt.colorbar(shrink=0.7)

fig.suptitle('Santa Cruz bathymetry from shipboard Multibeam')
plt.savefig('santacruz_multibeam_bathymetry.png', dpi=300)
plt.close(fig)
