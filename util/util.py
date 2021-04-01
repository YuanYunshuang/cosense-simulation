import pygame
import numpy as np
from PIL import Image
from matplotlib import cm
import open3d as o3d
import carla
import os
from util.color_encoding import LABEL_COLORS
cc = carla.ColorConverter.CityScapesPalette


#######################Sumo#############################
def write_sumocfg(path, junction):
    cfg_str = """<?xml version="1.0" encoding="UTF-8"?>
    <configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

        <input>
            <net-file value="Town05.net.xml"/>
            <route-files value="../carlavtypes.rou.xml,{}/route.rou.xml"/>
            <additional-files value="{}/rerouter.add.xml"/>
        </input>
    
        <gui_only>
            <gui-settings-file value="../viewsettings.xml"/>
        </gui_only>

    </configuration>
    """.format(junction, junction)
    with open(path, 'w') as f:
        f.write(cfg_str)

#######################Carla#############################
def get_blueprint(keywords, bp_lib, cfg):
    blp = bp_lib.find(keywords)
    # set attributes
    for k, v in cfg.items():
        if blp.has_attribute(k):
            blp.set_attribute(k, str(v))
    return blp

#######################Image#############################
def draw_save_image(surface, image, save_name=None, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    img_out = Image.fromarray(array)
    if save_name is not None:
        img_out.save(save_name)
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def carla_sensor_callback(data, sensor_queue, sensor_name, sensor_id, parent_id):
    sensor_queue.put((sensor_name, sensor_id, parent_id, data))


def image_callback(image, file_name=None, is_annotation=False):
    if is_annotation:
        image.convert(cc)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    if np.all(array==0):
        return
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    img_out = Image.fromarray(array)
    if file_name is not None:
        img_out.save(file_name)
    write_image_meta_info(file_name, image)
    # sensor_queue.put((image.frame, sensor_name))


#######################Point cloud#############################


VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])


def write_lidar_meta_info(filename, point_cloud):
    file_path, name = filename.rsplit('/', 1)
    name = name[:-4] + '_meta.txt'
    file_meta = os.path.join(file_path, name)
    with open(file_meta, 'w') as f:
        lines = []
        loc = point_cloud.transform.location
        rot = point_cloud.transform.rotation
        lines.append(','.join([str(point_cloud.frame),
               str(point_cloud.timestamp),
               str(point_cloud.horizontal_angle),
               str(point_cloud.channels)]))
        lines.append(','.join([
               str(rot.roll),str(rot.pitch),str(rot.yaw),
               str(loc.x),str(loc.y),str(loc.z)]))
        lines.append(','.join([str(point_cloud.get_point_count(channel)) for channel in range(point_cloud.channels)]))
        f.write('\n'.join(lines))


def write_image_meta_info(filename, image_data):
    file_path, name = filename.rsplit('/', 1)
    name = name[:-4] + '_meta.txt'
    file_meta = os.path.join(file_path, name)
    with open(file_meta, 'w') as f:
        lines = []
        loc = image_data.transform.location
        rot = image_data.transform.rotation
        lines.append(','.join([str(image_data.frame),
               str(image_data.timestamp),
               str(image_data.fov),
               str(image_data.height),
               str(image_data.width)]))
        lines.append(','.join([
               str(rot.roll),str(rot.pitch),str(rot.yaw),
               str(loc.x),str(loc.y),str(loc.z)]))
        f.write('\n'.join(lines))


def pointcloud_callback(point_cloud, file_name=None, blend=False):
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    if np.all(data==0):
        return
    data = np.reshape(data, (int(data.shape[0] / 4), 4))
    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Isolate the 3D data
    points = data[:, :-1]

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, :1] = -points[:, :1]

    # # An example of converting points from sensor to vehicle space if we had
    # # a carla.Transform variable named "tran":
    # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
    # points = np.dot(tran.get_matrix(), points.T).T
    # points = points[:, :-1]
    point_list = o3d.geometry.PointCloud()
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    o3d.io.write_point_cloud(file_name, point_list)
    # write meta data
    write_lidar_meta_info(file_name, point_cloud)

    # sensor_queue.put((point_cloud.frame, sensor_name))


def semantic_lidar_callback(point_cloud, file_name=None):
    """Prepares a point cloud with semantic segmentation
    colors ready to be consumed by Open3D"""
    data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points = np.array([data['x'], -data['y'], data['z']]).T
    if np.all(points==0):
        return
    # # An example of adding some noise to our data if needed:
    # points += np.random.uniform(-0.05, 0.05, size=points.shape)

    # Colorize the pointcloud based on the CityScapes color palette
    labels = np.array(data['ObjTag'])
    label_color = LABEL_COLORS / 255.0
    int_color = label_color[labels]

    # # In case you want to make the color intensity depending
    # # of the incident ray angle, you can use:
    # int_color *= np.array(data['CosAngle'])[:, None]
    point_list = o3d.geometry.PointCloud()
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    o3d.io.write_point_cloud(file_name, point_list)

    # write meta data
    write_lidar_meta_info(file_name, point_cloud)

    # sensor_queue.put((point_cloud.frame, sensor_name))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def generate_lidar_bp(arg, world, blueprint_library, delta):
    """Generates a CARLA blueprint based on the script parameters"""
    if arg.semantic:
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    else:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        if arg.no_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        else:
            lidar_bp.set_attribute('noise_stddev', '0.2')

    lidar_bp.set_attribute('upper_fov', str(arg.upper_fov))
    lidar_bp.set_attribute('lower_fov', str(arg.lower_fov))
    lidar_bp.set_attribute('channels', str(arg.channels))
    lidar_bp.set_attribute('range', str(arg.range))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(arg.points_per_second))
    return lidar_bp