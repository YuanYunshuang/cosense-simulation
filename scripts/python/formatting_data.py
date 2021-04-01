import os
import numpy as np
import glob
import open3d as o3d
import xml.etree.ElementTree as ET
import shutil
from util.color_encoding import LABEL_COLORS
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

Re2l = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64)
Te2l = np.array([0, 0, 0.3], dtype=np.float64)
tf_e2l = np.array([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0.3],
                   [0, 0, 0, 1]], dtype=np.float64)


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)


def rot2eul(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = np.pi/2.0
        psi = np.arctan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -np.pi/2.0
        psi = np.atan2(-R[0,1],-R[0,2])
    else:
        theta = -np.arcsin(R[2,0])
        cos_theta = np.cos(theta)
        psi = np.arctan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = np.arctan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return [psi, theta, phi]


def get_tf_matrix(rot, loc):
    # rot_mat = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(rot)
    rot_mat = R.from_rotvec(rot).as_matrix()
    tf_mat = np.zeros((4, 4), dtype=np.float64)
    tf_mat[:3, :3] = rot_mat
    tf_mat[:3, 3:4] = loc
    tf_mat[3, 3] = 1.0

    return tf_mat


def write_cloud_ego(cloud, out_file):
    points_ego_np = np.array(cloud.points).astype(np.float32)
    # find point label
    colors_ego_np = (np.array(cloud.colors) * 255).astype(np.uint8)
    labels = np.where(np.all(colors_ego_np[:, :, None] == LABEL_COLORS.T[None, :, :], axis=1))[1]
    point_array = np.concatenate([points_ego_np, labels.reshape(-1, 1).astype(np.float32)], axis=1)
    point_array.astype('float32').tofile(out_file)


def write_cloud_fused(clouds, cloud_ego, meta_info_list, meta_info_ego, out_path, voxel_size=0.1):
    tf_ego = get_tf_matrix(*meta_info_ego[2:4])
    tfs_dict = {'tf_ego': tf_ego}
    # transform clouds to the coordinate of ego lidar
    points_out = []
    points_out_colors = []
    points_out.append(np.array(cloud_ego.points))
    points_out_colors.append(np.array(cloud_ego.colors))

    for i, meta_info in enumerate(meta_info_list):
        tf = get_tf_matrix(*meta_info[2:4])
        v_id = meta_info[1]
        tfs_dict[v_id] = tf
        # transform clouds to ego-vehicle lidar frame
        cloud = clouds[i].transform(tf).transform(np.linalg.inv(tf_ego))
        # save original and transformed points of neighbors before fusing
        save_cloud_to_bin(cloud, out_path.format('cloud_coop_in_egoCS', v_id + '.bin'))
        save_cloud_to_bin(clouds[i], out_path.format('cloud_coop', v_id + '.bin'))
        points_out.append(np.array(cloud.points))
        points_out_colors.append(np.array(cloud.colors))

    # fuse and save
    pcd = o3d.geometry.PointCloud()
    point_array = np.concatenate(points_out, axis=0)
    point_colors_array = np.concatenate(points_out_colors, axis=0)
    pcd.points = o3d.utility.Vector3dVector(point_array)
    pcd.colors = o3d.utility.Vector3dVector(point_colors_array)
    bound = np.ones((3, 1), dtype=np.float64) * voxel_size
    cloud_fused, map, inds = o3d.geometry.PointCloud\
        .voxel_down_sample_and_trace(pcd, voxel_size, bound, bound)
    # write binary file
    save_cloud_to_bin(cloud_fused, out_path.format('cloud_fused', '.bin'))

    # save tfs
    np.save(out_path.format('tfs', '.npy'), tfs_dict)


def save_cloud_to_bin(cloud, filename):
    ## get points' coordinates
    points_np = np.array(cloud.points).astype(np.float32)
    ## get point-wise semantic label
    colors_np = (np.array(cloud.colors) * 255).astype(np.uint8)
    dists = np.abs(colors_np[:, :, None] - LABEL_COLORS.T[None, :, :]).sum(axis=1)
    labels = np.argmin(dists, axis=1).reshape(-1, 1)
    ## append label to the point as the 4-th element and add the labeled points to list
    points = np.concatenate([points_np, labels.astype(np.float32)], axis=1)
    ## write binary file
    points.astype('float32').tofile(filename)


def read_vtypes(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    vtypes_cls = {}
    vtypes_size = {}
    for vt in root.findall('vType'):
        vtypes_cls[vt.attrib['id']] = vt.attrib['vClass']
        vtypes_size[vt.attrib['id']] = [vt.attrib['length'],
                                        vt.attrib['width'],
                                        vt.attrib['height']]
    return vtypes_cls, vtypes_size


def read_meta_info(meta_file):
    id = meta_file.split('/')[-3]
    with open(meta_file, 'r') as f:
        line = f.readline().split(',')
        frame = line[0]
        line = f.readline().split(',')
        rot = - np.array([float(l) / 180 * np.pi for l in line[:3]], dtype=np.float64)
        # rot[2] = - rot[2]
        loc = np.array([[float(line[3])], [-float(line[4])], [float(line[5])]], dtype=np.float64)
        line = f.readline().split(',')
        channels = np.array([int(c) for c in line], dtype=np.int32)

        return frame, id, rot, loc, channels


def write_bbox(info_file, vtypes_file, out_path, ego_vehicle_id):
    vtypes_cls, vtypes_size = read_vtypes(vtypes_file)
    with open(info_file, 'r') as fh:
        infos = fh.readlines()
        infos.pop(0)
    # read vehicle info to dict
    info_dict = {}
    for s in infos:
        info = s.strip().split(',')
        if info[0] not in info_dict:
            info_dict[info[0]] = {}
        info_dict[info[0]][info[2]] = info[3:] + [info[1]]
    # change metric and tranform to ego-lidar CS
    for frame, data in info_dict.items():
        ids = []
        tfs = []
        v_classes = []
        v_sizes = []
        tf_ego = None

        for id, values in data.items():
            # values: x,y,z,rx,ry,rz,l,w,h,type_id
            type_id = values[-1]
            h = float(values[-2])
            ids.append(id)
            v_classes.append(vtypes_cls[type_id])
            v_size = [float(s) for s in values[6:9]]
            v_sizes.append(v_size)
            tf_g = np.array(values[:6]).astype(np.float)
            tf_g[3:] = - tf_g[3:] / 180 * np.pi
            # tf[1] = - tf[1]
            tf_g[2] = tf_g[2] + v_size[2] / 2
            tf_g[1] = - tf_g[1]
            if int(id)==int(ego_vehicle_id):
                tf_ego = tf_g
                T_ego2lidar = Te2l + np.array([0, 0, h / 2], dtype=np.float64)
                #tfs.append(np.array([0.0, 0.0, -h / 2 - 0.2, 0.0, 0.0, 0.0])) # 0.2 is lidar mounting height

            tfs.append(tf_g)

        if tf_ego is not None:
            rot_g2e = np.linalg.inv(o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(tf_ego[3:]))
            ss = '{} ' * 2 + '{:.3f} ' * 9 + '\n'
            with open(out_path + '_' + frame + '.txt', 'w') as flbl:
                for i, id  in enumerate(ids):
                    # if int(id)==int(ego_vehicle_id):
                    #     flbl.write(ss.format(id, v_classes[i], *tfs[i], *v_sizes[i]))
                    # else:
                    rot_b2g = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(tfs[i][3:])
                    rot_b2e2l = np.dot(rot_g2e, rot_b2g)
                    angles = rot2eul(rot_b2e2l)
                    loc_e = np.dot(rot_g2e, np.reshape(tfs[i][:3] - tf_ego[:3], (3,1)))
                    loc_l = (np.dot(Re2l, loc_e).squeeze() - T_ego2lidar).tolist()
                    flbl.write(ss.format(id, v_classes[i], *loc_l, *angles, *v_sizes[i]))


def write_ego_vehicle_info(in_path, out_path):
    files = glob.glob(in_path + '/*/*.ego')
    with open(os.path.join(out_path, 'ego_info.txt'), 'w') as fo:
        for file in files:
            junction = file.split('/')[-2][1:]
            with open(file, 'r') as fh:
                line = fh.readlines()[0]
            fo.write(junction + ',' + line + '\n')


def main(in_path, out_path, vtypes_file):
    # create dirs for saving data
    shutil.rmtree(out_path, ignore_errors=True)
    dirs = ['cloud_ego', 'cloud_fused', 'label_box',
            'cloud_coop', 'cloud_coop_in_egoCS', 'tfs']
    for d in dirs:
        os.makedirs(os.path.join(out_path, d))

    write_ego_vehicle_info(in_path, out_path)

    # get all junctions
    junctions = os.listdir(in_path)

    for junc in junctions:
        print('junction: %s \n' % junc)
        dirs = os.listdir(os.path.join(in_path, junc))
        vehicles = []
        ego_vehicle_id = None

        # find ego vehicle id, and list all other vehicle ids
        for d in dirs:
            if 'ego' in d:
                ego_vehicle_id = d[:-4].zfill(6)
            elif 'csv' not in d:
                vehicles.append(d)

        info_file = os.path.join(in_path, junc, 'info.csv')
        write_bbox(info_file, vtypes_file, os.path.join(out_path, 'label_box', junc[1:]), ego_vehicle_id)

        # find all valid frames of ego vehicle and the corresponding neighbors
        frames = glob.glob(os.path.join(in_path, junc, ego_vehicle_id, 'lidar_sem', '*.pcd'))

        for frame in tqdm(frames):
            filename = os.path.join(out_path, '{}', junc[1:] + '_'
                                    + frame.split('/')[-1][:-4] + '{}')
            pcd_ego = o3d.io.read_point_cloud(frame)
            meta_info_ego = read_meta_info(frame.replace('.pcd', '_meta.txt'))

            # write fused point clouds
            clouds = []
            meta_infos = []
            for v in vehicles:
                if v==ego_vehicle_id:
                    continue
                pcd_file = frame.rsplit('/', 4)
                pcd_file[-3] = v
                pcd_file = '/'.join(pcd_file)
                if os.path.exists(pcd_file):
                    clouds.append(o3d.io.read_point_cloud(pcd_file))
                    meta_infos.append(read_meta_info(pcd_file.replace('.pcd', '_meta.txt')))

            # write point clouds to binary files
            if len(clouds) > 0:
                write_cloud_ego(pcd_ego, filename.format('cloud_ego', '.bin'))
                os.makedirs(filename.format('cloud_coop', ''))
                os.makedirs(filename.format('cloud_coop_in_egoCS', ''))
                write_cloud_fused(clouds, pcd_ego, meta_infos, meta_info_ego, filename)


if __name__ == "__main__":
    cur_dir = os.path.dirname(__file__)
    in_path = "/media/hdd/yuan/koko/data/simulation"
    file_vtypes = os.path.join(cur_dir, "../../data/carlavtypes.rou.xml")
    out_path = "/media/hdd/yuan/koko/data/synthdata"
    main(in_path, out_path, file_vtypes)
