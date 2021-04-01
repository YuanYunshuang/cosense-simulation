
import cv2
import numpy as np
from pathlib import Path
import os
import tqdm
img_array = []
size = (300, 400)
path = '/media/hdd/ophelia/multi_ego1919'
data_path = Path('/media/hdd/ophelia/multi_ego1919')
vehicles = [x[0] for x in os.walk(path)]
images = [str(p) for p in list(data_path.rglob('*/camera/*.png'))]
frames = np.unique([f.split('/')[-1][:-4] for f in images])
frame_start = int(frames[0])
frame_end = int(frames[-1])

imgs_per_frame = {frame: [] for frame in range(frame_start, frame_end+1)}
ego_vehicle_id = '002044'
for i in images:
    imgs_per_frame[int(str(i).split('/')[-1][:-4])].append(i)

for frame in tqdm.tqdm(range(frame_start, frame_end)):
    img_names = imgs_per_frame[frame]
    video_frame = np.zeros((size[0]*2, size[1]*3, 3))
    win = {0: (0,0), 1: (0,1), 2: (0, 2), 3: (1, 0), 4: (1, 1), 5: (1, 2)}
    ego_filename = '/media/hdd/ophelia/multi_ego1919/002044/camera/%06d.png' % frame

    if os.path.exists(ego_filename):
        img = cv2.imread(ego_filename)[::2, ::2, :]
        video_frame[:size[0], :size[1], :] = img
        img_names.remove(ego_filename)
        
    for i, filename in enumerate(img_names):
        img = cv2.imread(filename)[::2, ::2, :]
        win_row = win[i+1][0]
        win_col = win[i+1][1]
        try:
            video_frame[size[0]*win_row:size[0]*(win_row + 1), size[1]*win_col:size[1]*(win_col + 1), :] = img
        except Exception as e:
            print(e)

    img_array.append(np.uint8(video_frame))

out = cv2.VideoWriter('project2044.avi', cv2.VideoWriter_fourcc(*'DIVX'), 10, (size[1]*3, size[0]*2))

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()