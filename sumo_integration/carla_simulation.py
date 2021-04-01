#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the carla simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import logging

import carla  # pylint: disable=import-error
from util.util import *
from .constants import INVALID_ACTOR_ID, SPAWN_OFFSET_Z
import queue
import os
import functools
import csv

# ==================================================================================================
# -- carla simulation ------------------------------------------------------------------------------
# ==================================================================================================


def _get_data_callback(sensor_name):
    try:
        return {
            'camera': image_callback,
            'camera_sem': functools.partial(image_callback, is_annotation=True),
            'lidar': pointcloud_callback,
            'lidar_sem': semantic_lidar_callback
        }[sensor_name]

    except:
        raise ("No sensor named {}.".format(sensor_name))


class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, host, port, step_length, cfg):
        self.client = carla.Client(host, port)
        self.client.set_timeout(60.0)

        self.world = self.client.load_world('Town05')
        self.spectator = self.world.get_spectator()
        spec_tf = self.spectator.get_transform()
        spec_tf.location.x = cfg['junc_coor'][0] - cfg['offset'][0]
        spec_tf.location.y = cfg['junc_coor'][1] - cfg['offset'][1]
        spec_tf.location.z = 100.0
        spec_tf.rotation.pitch = -90.0
        self.spectator.set_transform(spec_tf)


        # We need to save the settings to be able to recover them at the end
        # of the script to leave the server in the same state that we found it.
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()

        # We set CARLA syncronous mode
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = step_length
        self.world.apply_settings(settings)

        self.blueprint_library = self.world.get_blueprint_library()
        self.step_length = step_length
        self.cfg = cfg

        # The following sets contain updated information for the current frame.
        self.ego_vehicle_id = None
        self._active_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()
        # the dict for attached sensors,
        # Format: {actor_id1: [sensor_id1, sensor_id2, ...], sensor_id2: [sensor_id1, sensor_id2, ...], ...}
        self.spawned_actors2sensors = {}
        self.all_sensors = []
        # 4 Queues for camera, camera_sem, lidar, lidar_sem data respectively
        n_sensors_per_vehicle = len(cfg["sensor_names"])
        self.sensor_queues = [queue.Queue() for i in range(n_sensors_per_vehicle)]
        # self.world_queue = queue.Queue()
        # self.world_snapshot = None
        # self.world.on_tick(self.world_queue.put) # on tick can only be used in asynchronous mode

        self.fh = open(self.cfg['root_path'] + '/info.csv', mode='w')
        self.file_writer = csv.writer(self.fh, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.file_writer.writerow(['frame', 'vehicle_id', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                                   'length', 'width', 'height'])

        # Set traffic lights.
        self._tls = {}  # {landmark_id: traffic_light_actor}

        tmp_map = self.world.get_map()
        for landmark in tmp_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_ligth = self.world.get_traffic_light(landmark)
                if traffic_ligth is not None:
                    self._tls[landmark.id] = traffic_ligth
                else:
                    logging.warning('Landmark %s is not linked to any traffic light', landmark.id)

    def get_actor(self, actor_id):
        """
        Accessor for carla actor.
        """
        return self.world.get_actor(actor_id)

    # This is a workaround to fix synchronization issues when other carla clients remove an actor in
    # carla without waiting for tick (e.g., running sumo co-simulation and manual control at the
    # same time)
    def get_actor_light_state(self, actor_id):
        """
        Accessor for carla actor light state.

        If the actor is not alive, returns None.
        """
        try:
            actor = self.get_actor(actor_id)
            return actor.get_light_state()
        except RuntimeError:
            return None

    @property
    def traffic_light_ids(self):
        return set(self._tls.keys())

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        if landmark_id not in self._tls:
            return None
        return self._tls[landmark_id].state

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(True)
                # We set the traffic light to 'green' because 'off' state sets the traffic light to
                # 'red'.
                actor.set_state(carla.TrafficLightState.Green)

    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new actor.

            :param blueprint: blueprint of the actor to be spawned.
            :param transform: transform where the actor will be spawned.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        transform = carla.Transform(transform.location + carla.Location(0, 0, SPAWN_OFFSET_Z),
                                    transform.rotation)

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False))
        ]
        response = self.client.apply_batch_sync(batch, False)[0]
        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID

        return response.actor_id

    def spawn_sensors_for(self, vehicle_id):
        sensor_names = self.cfg["sensor_names"]
        sensors_list = []
        if vehicle_id not in self.spawned_actors2sensors:
            vehicle = self.world.get_actor(vehicle_id)
            vehicle_height = vehicle.bounding_box.extent.z * 2

            batch_sensors = []
            # get sensor blueprints
            blueprints_sensor = self.blueprint_library.filter('sensor')
            # get sensor blueprints and transform
            for sensor in sensor_names:
                if 'lidar' in sensor:
                    tf = self.cfg['lidars'][0]['transform']
                    if 'sem' in sensor:
                        bp = get_blueprint('sensor.lidar.ray_cast_semantic', blueprints_sensor,
                                                 self.cfg['lidars'][0])
                    else:
                        bp = get_blueprint('sensor.lidar.ray_cast', blueprints_sensor,
                                           self.cfg['lidars'][0])
                elif 'camera' in sensor:
                    tf = self.cfg['cameras'][0]['transform']
                    if 'sem' in sensor:
                        bp = get_blueprint('sensor.camera.semantic_segmentation', blueprints_sensor,
                                           self.cfg['cameras'][0])
                    else:
                        bp = get_blueprint('sensor.camera.rgb', blueprints_sensor,
                                           self.cfg['cameras'][0])
                else:
                    raise ValueError("sensor type \"%s\" is not supported." % sensor)
                tf = carla.Transform(carla.Location(x=tf[0], y=tf[1], z=tf[2] + vehicle_height),
                                           carla.Rotation(roll=tf[3], pitch=tf[4], yaw=tf[5]))

                # generate cmd for spawning sensor
                batch_sensors.append(carla.command.SpawnActor(bp, tf, vehicle))

            for response in self.client.apply_batch_sync(batch_sensors, False):
                if response.error:
                    logging.error(response.error)
                else:
                    sensors_list.append(self.world.get_actor(response.actor_id))
            # Make folders for storing data of this vehicle
            data_path = os.path.join(self.cfg['root_path'], '%06d' % vehicle_id)

            if not os.path.exists(data_path):
                for name in sensor_names:
                    os.makedirs(os.path.join(data_path, name))
            self.all_sensors.extend(sensors_list)
            self.spawned_actors2sensors[vehicle_id] = sensors_list
        else:
            sensors_list = self.spawned_actors2sensors[vehicle_id]

        for i in range(len(sensor_names)):
            # you cannot use the same callback function for different sensors, otherwise all sensors would use the
            # same configuration for all callbacks passed to the sensors. Use functiontools to wrap the same callback
            # and fix some arguments with different parameters and force the same callback function to be different
            # callback functions.
            # self.sensor_queues[vehicle_id] = [queue.Queue() for i in range(4)]
            sensors_list[i].listen(functools.partial(carla_sensor_callback,
                                                      sensor_queue=self.sensor_queues[i],
                                                      sensor_name=sensor_names[i],
                                                      sensor_id=sensors_list[i].id,
                                                      parent_id=vehicle_id))

    def destroy_sensors_for(self, vehicle_id):
        for sensor in self.spawned_actors2sensors[vehicle_id]:
            sensor.destroy() # This will trigger an error in streaming and mess up the data order
        self.spawned_actors2sensors.pop(vehicle_id)

    def stop_sensors_for(self, vehicle_id):
        for sensor in self.spawned_actors2sensors[vehicle_id]:
            sensor.stop()

    def destroy_all_actors(self):
        for sensor in self.all_sensors:
            sensor.destroy()
        for vehicle_id in self._active_actors:
            self.destroy_actor(vehicle_id)

        # self.spawned_actors2sensors = {}

    def num_of_sensors(self):
        n_sensors = 0
        for vehicle, sensors in self.spawned_actors2sensors.items():
            for _ in sensors:
                n_sensors = n_sensors + 1
        return n_sensors

    def destroy_actor(self, actor_id):
        """
        Destroys the given actor
        """
        actor = self.world.get_actor(actor_id)
        if actor is not None:
            actor.destroy()

    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param lights: new vehicle light state.
            :return: True if successfully updated. Otherwise, False.
        """
        vehicle = self.world.get_actor(vehicle_id)
        if vehicle is None:
            return False

        vehicle.set_transform(transform)
        if lights is not None:
            vehicle.set_light_state(carla.VehicleLightState(lights))
        return True

    def synchronize_sensors(self):
        for k, v in self.spawned_actors2sensors.items():
            for j in range(len(v)):
                if not self.sensor_queues[j].empty():
                    sensor_name, sensor_id, vehicle_id, data = self.sensor_queues[j].get(True, timeout=60.0)
                    if self.world.get_actor(sensor_id) is not None and self.world.get_actor(vehicle_id) is not None:
                        ext = 'png' if 'camera' in sensor_name else 'pcd'
                        callback = _get_data_callback(sensor_name)
                        callback(data, file_name=self.cfg['root_path'] + '/%06d/' % vehicle_id
                                                 + sensor_name + '/%06d.' % data.frame + ext)
                    else:
                        raise ValueError("synchronize_sensors: sensor or vehicle not found.")

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param landmark_id: id of the landmark to be updated.
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        if not landmark_id in self._tls:
            logging.warning('Landmark %s not found in carla', landmark_id)
            return False

        traffic_light = self._tls[landmark_id]
        traffic_light.set_state(state)
        return True

    def tick(self):
        """
        Tick to carla simulation.
        """
        self.world.tick()
        # self.world_snapshot = self.world_queue.get(timeout=60.0)
        # Update data structures for the current frame.
        current_actors = set(
            [vehicle.id for vehicle in self.world.get_actors().filter('vehicle.*')])
        self.spawned_actors = current_actors.difference(self._active_actors)
        self.destroyed_actors = self._active_actors.difference(current_actors)
        self._active_actors = current_actors

        world_snapshot = self.world.get_snapshot()
        for v in current_actors:
            line = '{:06},{},{:d}' + ',{:.3f}' * 9
            tf = world_snapshot.find(v).get_transform()
            vehicle = self.world.get_actor(v)
            extent = vehicle.bounding_box.extent
            type_id = vehicle.type_id
            line = line.format(world_snapshot.frame, type_id, v,
                        tf.location.x, tf.location.y, tf.location.z,
                        tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw,
                        extent.x * 2, extent.y * 2, extent.z * 2)
            self.file_writer.writerow(line.split(','))

        self.synchronize_sensors()

    def close(self):
        """
        Closes carla client.
        """
        self.fh.close()
        self.destroy_all_actors()
        # for actor in self.world.get_actors():
        #     if actor.type_id == 'traffic.traffic_light':
        #         actor.freeze(False)
        #     elif actor.is_alive and ('vehicle' in actor.type_id or 'sensor' in actor.type_id):
        #         actor.destroy()
        self.world.apply_settings(self.original_settings)
        left_sensors = len(self.world.get_actors().filter('sensor*'))
        left_vehicles = len(self.world.get_actors().filter('vehicle*'))
        if left_sensors + left_vehicles > 0:
            print('Some actors still left in carla world when shutting down:')
            print('  - sensors :', left_sensors)
            print('  - vehicles:', left_vehicles)


