import glob
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import xarray as xr
import gc

import matplotlib
matplotlib.rc('font', size=12)

data_path = 'processed_netcdf'

multibeam_files = glob.glob(data_path + '/*.nc')
multibeam_files.sort()

lon0, lon1 = -122.15, -121.95
lat0, lat1 = 36.9, 37.

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

skip = 4

for f in sc_files:
    print('Plotting ', f)
    ds = xr.open_dataset(f)
    lon = np.array(ds.longitude[::skip,::skip])
    lat = np.array(ds.latitude[::skip,::skip])
    depth = np.array(ds.depth[::skip,::skip])

    fig = plt.figure(figsize=(8, 6))
    plt.contourf(lon, lat, depth)
    plt.colorbar()
    plt.title(f)
    plt.savefig(f + '.png')
    plt.close()
