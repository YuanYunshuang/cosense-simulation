import glob
import os
import sys

try:
    sys.path.append(glob.glob('/media/hdd/ophelia/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from queue import Queue, Empty
from util.util import image_callback

def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    sensor_queue.put((sensor_data, sensor_name))
    sensor_queue.put((sensor_data, sensor_name))


def main():
    # We start creating the client
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    original_settings = world.get_settings()
    settings = world.get_settings()

    # We set CARLA syncronous mode
    settings.fixed_delta_seconds = 0.5
    settings.synchronous_mode = True
    world.apply_settings(settings)
    try:
        sensors = world.get_actors().filter('sensor*')
        for s in sensors:
            print('Sensor: ', s.id, s.destroy())
        vehicles = world.get_actors().filter('vehicle*')
        for v in vehicles:
            print('Vehicle:', v.id, v.destroy())

        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        sensor_queue = Queue()

        # Bluepints for the sensors
        blueprint_library = world.get_blueprint_library()
        cam_bp = blueprint_library.find('sensor.camera.rgb')

        # We create all the sensors and keep them in a list for convenience.
        sensor_list = []

        cam01 = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=50, y=50, z=3),
                                 carla.Rotation(roll=0, pitch=0, yaw=0)))
        cam01.listen(lambda data: sensor_callback(data, sensor_queue, "camera01"))
        sensor_list.append(cam01)

        cam02 = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=5, y=5, z=3),
                                 carla.Rotation(roll=0, pitch=0, yaw=90)))
        cam02.listen(lambda data: sensor_callback(data, sensor_queue, "camera02"))
        sensor_list.append(cam02)

        cam03 = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=50, y=5, z=3),
                                 carla.Rotation(roll=0, pitch=0, yaw=180)))
        cam03.listen(lambda data: sensor_callback(data, sensor_queue, "camera03"))
        sensor_list.append(cam03)


        for ep in range(10):
            try:
                world.tick()
                if ep==3:
                   #cam01.stop()
                   cam01.destroy()
                   sensor_list.remove(cam01)

                #for i in range(0, len(sensor_list)):
                while not sensor_queue.empty():
                    data, name = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" % (data.frame, name))
                    image_callback(data, file_name='/media/hdd/ophelia/carla/Co-Simulation/Sumo/tmp/{}/{}.png'.format(name, data.frame))


            except Empty:
                print("    Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
        for sensor in sensor_list:
            sensor.destroy()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
