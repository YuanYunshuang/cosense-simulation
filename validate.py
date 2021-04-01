from pathlib import Path
import os
from scripts.python.formatting_data import read_meta_info

path = Path('/media/hdd/yuan/koko/data/simulation2/j1148')
# find all valid frames of ego vehicle and the corresponding neighbors
info_file = (path / 'info.csv')
with info_file.open() as fh:
    infos = fh.readlines()
    infos.pop(0)
info_dict = {}
for s in infos:
    info = s.strip().split(',')
    if info[0] in info_dict:
        info_dict[info[0]][info[2]] = info[3:]
    else:
        info_dict[info[0]] = {}
        info_dict[info[0]][info[2]] = info[3:]
ego = '000255'
listdirs = path.glob('*/')
vehicles = []
for d in listdirs:
    if 'csv' not in str(d):
        vehicles.append(d.name)

for frame_file in (path / ego).glob('lidar_sem/*.txt'):
    frame_file_nbr = str(frame_file).rsplit('/', 3)
    frame = frame_file_nbr[-1][:-9]
    for v in vehicles:
        frame_file_nbr[1] = v
        if os.path.exists('/'.join(frame_file_nbr)):
            metas = read_meta_info('/'.join(frame_file_nbr))
            sensor_height = float(metas[2][2])
            vehicle_height = float(info_dict[frame][v[3:]][-1])
            location_height = float(info_dict[frame][v[3:]][2])
            if sensor_height-(vehicle_height+location_height+0.1) > 0.02:
                raise ValueError('Error found in height information: \n '
                                 'sensor: {:.3f}, vehicle: {:.3f}, location: {:.3f}'.format(
                                sensor_height,
                                vehicle_height,
                                location_height
                                ))
print("Height information in all frames are correct!")


