from datetime import datetime
import numpy as np
import os
import xarray as xr


def pings_to_netcdf(pings_file_path: str):
    """Converts a Multibeam EM-712 pings file to NetCDF.
    """
    
    data = [line.strip() for line in open(pings_file_path).readlines()[1:]]
    headers = data[::2]
    point_data = data[1::2]

    num_scans = len(headers)
    scan_size = 400

    longitude = np.zeros((num_scans, scan_size), dtype=np.float64)
    latitude = np.zeros((num_scans, scan_size), dtype=np.float64)
    depth = np.zeros((num_scans, scan_size), dtype=np.float32)
    xerr = np.zeros((num_scans, scan_size), dtype=np.float32)
    zerr = np.zeros((num_scans, scan_size), dtype=np.float32)
    time = np.zeros((num_scans), dtype=np.float64)

    for n in range(num_scans):

        # Parse header
        lat, lon, zref_ellipsoid, zref_waterlevel, ms = [float(x) for x in headers[n].split()]
        time[n] = ms * 1e-3

        # Parse point cloud data
        point_data_record = point_data[n].split()
        
        if len(point_data_record) < 2000:
            # bad record; assign NaN and skip
            print('Bad record ', n)
            latitude[n] = np.nan
            longitude[n] = np.nan
            depth[n] = np.nan
            xerr[n] = np.nan
            xerr[n] = np.nan
            continue

        dlat = np.array([float(x) for x in point_data_record[::5]])
        dlon = np.array([float(x) for x in point_data_record[1::5]])
        zref = np.array([float(x) for x in point_data_record[2::5]])
        zerr[n] = np.array([float(x) for x in point_data_record[3::5]])
        xerr[n] = np.array([float(x) for x in point_data_record[4::5]])

        # zref is relative to zref_waterlevel;
        # subtract it to get the water depth
        depth[n] = zref - zref_waterlevel
        latitude[n] = lat + dlat
        longitude[n] = lon + dlon

    ds = xr.Dataset(
        {
            'time': (['scans'], time),
            'latitude': (['scans', 'points'], latitude),
            'longitude': (['scans', 'points'], longitude),
            'depth': (['scans', 'points'], depth),
            'horizontal_error': (['scans', 'points'], xerr),
            'vertical_error': (['scans', 'points'], zerr),
        },
        coords = {
            'scans': time,
            'points': np.array(range(scan_size), dtype=np.float32)
        }
    )

    output_filename = '.'.join(os.path.basename(pings_file_path).split('.')[:-1] + ['nc']) 
    ds.to_netcdf(output_filename, 'w', 'NETCDF4')
