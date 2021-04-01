import json
import shutil
import argparse
import glob
import os
import sys
import logging
import time
# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

carla_path = glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
              (os.environ['CARLA_HOME'], sys.version_info.major, sys.version_info.minor,
               'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0]
if len(carla_path)>0:
    sys.path.append(carla_path)
    print("'%s' is add to system path." % carla_path)
else:
    raise ImportError('carla not found.')
# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
# ==================================================================================================
# -- import project modules ------------------------------------------------------------------------
# ==================================================================================================
from util.util import write_sumocfg
from run_synchronization import synchronization_loop

# ==================================================================================================
# -- main-------------------------------------------------------------------------------------------
# ==================================================================================================
def pharse_args():
    argparser = argparse.ArgumentParser(description=__doc__)
    # argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file')
    argparser.add_argument('--sensor-cfg-file',
                            metavar='C',
                            default='config.json',
                            help='Path for config files')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to liston to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.1,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    args = argparser.parse_args()

    return args


if __name__ == '__main__':
    args = pharse_args()

    # Load config file
    with open(args.sensor_cfg_file) as json_file:
        sensor_cfg = json.load(json_file)

    if args.sync_vehicle_all is True:
        args.sync_vehicle_lights = True
        args.sync_vehicle_color = True

    if args.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    with open(sensor_cfg['junction_info']) as junc_file:
        junctions_info = json.load(junc_file)
    root_path = sensor_cfg['root_path']
    # junctions = [x for x in os.walk(os.path.dirname(sensor_cfg['sumocfg']))][0][1]

    for junc_id, junc_coor in junctions_info.items():
        print("=============SIMULATION IN JUNCTION %s============" % junc_id)
        # make dirs for saving data
        data_path = os.path.join(root_path, "j" + junc_id)
        sensor_cfg['root_path'] = data_path
        shutil.rmtree(data_path, ignore_errors=True)
        os.makedirs(data_path)

        # write sumo config file
        write_sumocfg(sensor_cfg['sumocfg'], junc_id)
        sensor_cfg['junc_coor'] = junc_coor
        synchronization_loop(args, sensor_cfg)
        time.sleep(1.0)