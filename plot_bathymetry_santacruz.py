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

lon0, lon1 = -122.12, -122
lat0, lat1 = 36.9, 37.

parallels = np.arange(lat0, lat1 + 0.02, 0.02)
meridians = np.arange(lon0, lon1 + 0.02, 0.02)

fig = plt.figure(figsize=(8, 6))
map = Basemap(llcrnrlon=lon0, llcrnrlat=lat0, urcrnrlon=lon1, urcrnrlat=lat1, \
              resolution='f')
map.drawcoastlines()
map.drawparallels(parallels, labels=~np.isnan(parallels))
map.drawmeridians(meridians, labels=~np.isnan(meridians))

sc_files = []
for f in multibeam_files:
    print('Reading', f)
    ds = xr.open_dataset(f)
    lon = np.array(ds.longitude)
    lat = np.array(ds.latitude)
    if np.max(lat) < lat0 or np.max(lon) < lon0 or np.min(lon) > lon1:
        print('Skipping')
        continue
    else:
        sc_files.append(f)

skip = 2

for f in sc_files:
    print('Plotting ', f)
    ds = xr.open_dataset(f)
    lon = np.array(ds.longitude[::skip,::skip])
    lat = np.array(ds.latitude[::skip,::skip])
    depth = np.array(ds.depth[::skip,::skip])

    plt.pcolor(lon, lat, depth, vmin=0, vmax=60, cmap=cm.viridis_r)
    del lon, lat, depth, ds
    gc.collect()

plt.colorbar()

fig.suptitle('Santa Cruz bathymetry from shipboard Multibeam EM-712')
plt.savefig('santacruz_multibeam_bathymetry.png', dpi=300)
plt.close(fig)
