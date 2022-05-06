from multibeam_em712.convert import pings_to_netcdf
import glob

ping_files = glob.glob('processed_data/*.pings')
ping_files.sort()

for ping_file in ping_files:
    print(ping_file)
    pings_to_netcdf(ping_file)
