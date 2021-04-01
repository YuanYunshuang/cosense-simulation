import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os
from glob import glob


def draw_points_2d(file_name):
    cloud = o3d.io.read_point_cloud(file_name)
    points = np.array(cloud.points)
    plt.plot(points[:, 0], points[:, 1], 'b.', markersize=0.5)
    plt.savefig('/media/hdd/yuan/cosense-simulation/cosim_v2/tmp.png')
    plt.close()


if __name__ == "__main__":
    path = "/media/hdd/yuan/koko/data/simulation/j1050/003049/lidar_sem"
    for file in glob(path + "/*.pcd"):
        draw_points_2d(file)
