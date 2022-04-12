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

multibeam_files = glob.glob(data_path + '/*.nc')
multibeam_files.sort()

lon0, lon1 = -122.2, -121.7
lat0, lat1 = 36.6, 37.

parallels = np.arange(lat0, lat1 + 0.1, 0.1)
meridians = np.arange(lon0, lon1 + 0.1, 0.1)

fig = plt.figure(figsize=(8, 6))
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
    plt.pcolor(lon, lat, depth, vmin=0, vmax=100, cmap=cm.viridis_r)
    del lon, lat, depth, ds
    gc.collect()

plt.colorbar()

fig.suptitle('Monterey Bay bathymetry from shipboard Multibeam EM-712')
plt.savefig('monterey_bay_multibeam_bathymetry.png', dpi=300)
plt.close(fig)
