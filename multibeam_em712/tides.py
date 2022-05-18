from datetime import datetime
from typing import List, Tuple


def read_tides(path: str) -> Tuple[List, List]:
    """Read a CO-OPS tide data CSV file and return
    a list of times and elevations."""
    data = [line.strip() for line in open(path)]
    time = []
    elevation = []
    for line in data[1:]:
        line = line.replace('"','').split(',')
        time.append(datetime.strptime(line[0] + line[1], '%Y/%m/%d%H:%M'))
        elevation.append(float(line[4]))
    return time, elevation
