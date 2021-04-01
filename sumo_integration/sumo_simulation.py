#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the sumo simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import collections
import enum
import logging
import os
import numpy as np

import carla  # pylint: disable=import-error
import sumolib  # pylint: disable=import-error
import traci  # pylint: disable=import-error
import traci.constants as tc

from .constants import INVALID_ACTOR_ID

import lxml.etree as ET  # pylint: disable=import-error

# ==================================================================================================
# -- sumo definitions ------------------------------------------------------------------------------
# ==================================================================================================


# https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#signal_state_definitions
class SumoSignalState(object):
    """
    SumoSignalState contains the different traffic light states.
    """
    RED = 'r'
    YELLOW = 'y'
    GREEN = 'G'
    GREEN_WITHOUT_PRIORITY = 'g'
    GREEN_RIGHT_TURN = 's'
    RED_YELLOW = 'u'
    OFF_BLINKING = 'o'
    OFF = 'O'


# https://sumo.dlr.de/docs/TraCI/Vehicle_Signalling.html
class SumoVehSignal(object):
    """
    SumoVehSignal contains the different sumo vehicle signals.
    """
    BLINKER_RIGHT = 1 << 0
    BLINKER_LEFT = 1 << 1
    BLINKER_EMERGENCY = 1 << 2
    BRAKELIGHT = 1 << 3
    FRONTLIGHT = 1 << 4
    FOGLIGHT = 1 << 5
    HIGHBEAM = 1 << 6
    BACKDRIVE = 1 << 7
    WIPER = 1 << 8
    DOOR_OPEN_LEFT = 1 << 9
    DOOR_OPEN_RIGHT = 1 << 10
    EMERGENCY_BLUE = 1 << 11
    EMERGENCY_RED = 1 << 12
    EMERGENCY_YELLOW = 1 << 13


# https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#abstract_vehicle_class
class SumoActorClass(enum.Enum):
    """
    SumoActorClass enumerates the different sumo actor classes.
    """
    IGNORING = "ignoring"
    PRIVATE = "private"
    EMERGENCY = "emergency"
    AUTHORITY = "authority"
    ARMY = "army"
    VIP = "vip"
    PEDESTRIAN = "pedestrian"
    PASSENGER = "passenger"
    HOV = "hov"
    TAXI = "taxi"
    BUS = "bus"
    COACH = "coach"
    DELIVERY = "delivery"
    TRUCK = "truck"
    TRAILER = "trailer"
    MOTORCYCLE = "motorcycle"
    MOPED = "moped"
    BICYCLE = "bicycle"
    EVEHICLE = "evehicle"
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    RAIL_FAST = "rail_fast"
    SHIP = "ship"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"


SumoActor = collections.namedtuple('SumoActor', 'type_id vclass transform signals extent color')

# ==================================================================================================
# -- sumo traffic lights ---------------------------------------------------------------------------
# ==================================================================================================


class SumoTLLogic(object):
    """
    SumoTLLogic holds the data relative to a traffic light in sumo.
    """
    def __init__(self, tlid, states, parameters):
        self.tlid = tlid
        self.states = states

        self._landmark2link = {}
        self._link2landmark = {}
        for link_index, landmark_id in parameters.items():
            # Link index information is added in the parameter as 'linkSignalID:x'
            link_index = int(link_index.split(':')[1])

            if landmark_id not in self._landmark2link:
                self._landmark2link[landmark_id] = []
            self._landmark2link[landmark_id].append((tlid, link_index))
            self._link2landmark[(tlid, link_index)] = landmark_id

    def get_number_signals(self):
        """
        Returns number of internal signals of the traffic light.
        """
        if len(self.states) > 0:
            return len(self.states[0])
        return 0

    def get_all_signals(self):
        """
        Returns all the signals of the traffic light.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return [(self.tlid, i) for i in range(self.get_number_signals())]

    def get_all_landmarks(self):
        """
        Returns all the landmarks associated with this traffic light.
        """
        return self._landmark2link.keys()

    def get_associated_signals(self, landmark_id):
        """
        Returns all the signals associated with the given landmark.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return self._landmark2link.get(landmark_id, [])


class SumoTLManager(object):
    """
    SumoTLManager is responsible for the management of the sumo traffic lights (i.e., keeps control
    of the current program, phase, ...)
    """
    def __init__(self):
        self._tls = {}  # {tlid: {program_id: SumoTLLogic}
        self._current_program = {}  # {tlid: program_id}
        self._current_phase = {}  # {tlid: index_phase}

        for tlid in traci.trafficlight.getIDList():
            self.subscribe(tlid)

            self._tls[tlid] = {}
            for tllogic in traci.trafficlight.getAllProgramLogics(tlid):
                states = [phase.state for phase in tllogic.getPhases()]
                parameters = tllogic.getParameters()
                tl = SumoTLLogic(tlid, states, parameters)
                self._tls[tlid][tllogic.programID] = tl

            # Get current status of the traffic lights.
            self._current_program[tlid] = traci.trafficlight.getProgram(tlid)
            self._current_phase[tlid] = traci.trafficlight.getPhase(tlid)

        self._off = False

    @staticmethod
    def subscribe(tlid):
        """
        Subscribe the given traffic ligth to the following variables:

            * Current program.
            * Current phase.
        """
        traci.trafficlight.subscribe(tlid, [
            tc.TL_CURRENT_PROGRAM,
            tc.TL_CURRENT_PHASE,
        ])

    @staticmethod
    def unsubscribe(tlid):
        """
        Unsubscribe the given traffic ligth from receiving updated information each step.
        """
        traci.trafficlight.unsubscribe(tlid)

    def get_all_signals(self):
        """
        Returns all the traffic light signals.
        """
        signals = set()
        for tlid, program_id in self._current_program.items():
            signals.update(self._tls[tlid][program_id].get_all_signals())
        return signals

    def get_all_landmarks(self):
        """
        Returns all the landmarks associated with a traffic light in the simulation.
        """
        landmarks = set()
        for tlid, program_id in self._current_program.items():
            landmarks.update(self._tls[tlid][program_id].get_all_landmarks())
        return landmarks

    def get_all_associated_signals(self, landmark_id):
        """
        Returns all the signals associated with the given landmark.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        signals = set()
        for tlid, program_id in self._current_program.items():
            signals.update(self._tls[tlid][program_id].get_associated_signals(landmark_id))
        return signals

    def get_state(self, landmark_id):
        """
        Returns the traffic light state of the signals associated with the given landmark.
        """
        states = set()
        for tlid, link_index in self.get_all_associated_signals(landmark_id):
            current_program = self._current_program[tlid]
            current_phase = self._current_phase[tlid]

            tl = self._tls[tlid][current_program]
            states.update(tl.states[current_phase][link_index])

        if len(states) == 1:
            return states.pop()
        elif len(states) > 1:
            logging.warning('Landmark %s is associated with signals with different states',
                            landmark_id)
            return SumoSignalState.RED
        else:
            return None

    def set_state(self, landmark_id, state):
        """
        Updates the state of all the signals associated with the given landmark.
        """
        for tlid, link_index in self.get_all_associated_signals(landmark_id):
            traci.trafficlight.setLinkState(tlid, link_index, state)
        return True

    def switch_off(self):
        """
        Switch off all traffic lights.
        """
        for tlid, link_index in self.get_all_signals():
            traci.trafficlight.setLinkState(tlid, link_index, SumoSignalState.OFF)
        self._off = True

    def tick(self):
        """
        Tick to traffic light manager
        """
        if self._off is False:
            for tl_id in traci.trafficlight.getIDList():
                results = traci.trafficlight.getSubscriptionResults(tl_id)
                current_program = results[tc.TL_CURRENT_PROGRAM]
                current_phase = results[tc.TL_CURRENT_PHASE]

                if current_program != 'online':
                    self._current_program[tl_id] = current_program
                    self._current_phase[tl_id] = current_phase


# ==================================================================================================
# -- sumo simulation -------------------------------------------------------------------------------
# ==================================================================================================

def _get_sumo_net(cfg_file):
    """
    Returns sumo net.

    This method reads the sumo configuration file and retrieve the sumo net filename to create the
    net.
    """
    cfg_file = os.path.join(os.getcwd(), cfg_file)

    tree = ET.parse(cfg_file)
    tag = tree.find('//net-file')
    if tag is None:
        return None

    net_file = os.path.join(os.path.dirname(cfg_file), tag.get('value'))
    logging.debug('Reading net file: %s', net_file)

    sumo_net = traci.sumolib.net.readNet(net_file)
    return sumo_net


class SumoSimulation(object):
    """
    SumoSimulation is responsible for the management of the sumo simulation.
    """
    def __init__(self,
                 cfg_file,
                 step_length,
                 host=None,
                 port=None,
                 sumo_gui=False,
                 client_order=1,
                 ego_vehicle_id='0',
                 comm_range=50.0):
        if sumo_gui is True:
            sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            sumo_binary = sumolib.checkBinary('sumo')

        if host is None or port is None:
            logging.info('Starting new sumo server...')
            if sumo_gui is True:
                logging.info('Remember to press the play button to start the simulation')

            traci.start([sumo_binary,
                '--configuration-file', cfg_file,
                '--step-length', str(step_length),
                '--lateral-resolution', '0.25',
                '--collision.check-junctions'
            ])

        else:
            logging.info('Connection to sumo server. Host: %s Port: %s', host, port)
            traci.init(host=host, port=port)

        traci.setOrder(client_order)

        # Retrieving net from configuration file.
        self.net = _get_sumo_net(cfg_file)

        # Creating a random route to be able to spawn carla actors.
        traci.route.add("carla_route", [traci.edge.getIDList()[0]])

        # Variable to asign an id to new added actors.
        self._sequential_id = 0

        # Ego vehicle id and state
        assert isinstance(ego_vehicle_id, str), 'ego vehicle id should be string'
        self.ego_vehicle = ego_vehicle_id
        self.ego_vehicle_state = 0  # 0: not departed yet, 1: departed, 2: arrived
        self.comm_range = comm_range

        # Structures to keep track of the spawned, destroyed vehicles and the vehicles that are in the range of
        # ego vehicle at each time step.
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self.inrange_actors = set()
        self.sensor_to_spawn = set()    # actors that should spawn sensors
        self.sensor_to_stop = set()  # actors on which the attached sensors should be destroyed
        self.perception_actors = set()

        # Traffic light manager.
        self.traffic_light_manager = SumoTLManager()

        # record num of ticks since the ego vehicle is alive
        self.ticks = 0

    @property
    def traffic_light_ids(self):
        return self.traffic_light_manager.get_all_landmarks()

    @staticmethod
    def subscribe(actor_id):
        """
        Subscribe the given actor to the following variables:

            * Type.
            * Vehicle class.
            * Color.
            * Length, Width, Height.
            * Position3D (i.e., x, y, z).
            * Angle, Slope.
            * Speed.
            * Lateral speed.
            * Signals.
        """
        traci.vehicle.subscribe(actor_id, [
            tc.VAR_TYPE, tc.VAR_VEHICLECLASS, tc.VAR_COLOR,
            tc.VAR_LENGTH, tc.VAR_WIDTH, tc.VAR_HEIGHT,
            tc.VAR_POSITION3D, tc.VAR_ANGLE, tc.VAR_SLOPE,
            tc.VAR_SPEED, tc.VAR_SIGNALS
        ])

    @staticmethod
    def unsubscribe(actor_id):
        """
        Unsubscribe the given actor from receiving updated information each step.
        """
        traci.vehicle.unsubscribe(actor_id)

    def subscribe_context(self, actor_id=None):
        """
        Subscribe to the neighbors of a given actor, if the actor id is not given then the ego vehicle of this Object
        will be used. The default maximum distance of the neighbors to the given actor is 25.0m.
        ---------
        @param actor_id: the center actor of the context
        @param distance: the maximum distance of the neighbors to the given actor
        """
        if actor_id is None:
            traci.vehicle.subscribeContext(self.ego_vehicle, tc.CMD_GET_VEHICLE_VARIABLE, self.comm_range,
                                           [tc.VAR_ANGLE, tc.VAR_POSITION, tc.VAR_VEHICLECLASS])
        else:
            traci.vehicle.subscribeContext(actor_id, tc.CMD_GET_VEHICLE_VARIABLE, self.comm_range,
                                           [tc.VAR_ANGLE, tc.VAR_POSITION, tc.VAR_VEHICLECLASS])

    def unsubscribe_context(self, actor_id=None):
        """
        Unsubscribe to the neighbors of a given actor, if the actor id is not given then the ego vehicle of this Object
        will be used.
        ---------
        @param actor_id: the center actor of the context
        """
        if actor_id is None:
            traci.vehicle.unsubscribeContext(self.ego_vehicle, tc.CMD_GET_VEHICLE_VARIABLE, self.comm_range)
        else:
            traci.vehicle.unsubscribeContext(actor_id)

    def get_net_offset(self):
        """
        Accessor for sumo net offset.
        """
        if self.net is None:
            return (0, 0)
        return self.net.getLocationOffset()

    @staticmethod
    def get_actor(actor_id):
        """
        Accessor for sumo actor.
        """
        results = traci.vehicle.getSubscriptionResults(actor_id)

        type_id = results[tc.VAR_TYPE]
        vclass = SumoActorClass(results[tc.VAR_VEHICLECLASS])
        color = results[tc.VAR_COLOR]

        length = results[tc.VAR_LENGTH]
        width = results[tc.VAR_WIDTH]
        height = results[tc.VAR_HEIGHT]

        location = list(results[tc.VAR_POSITION3D])
        rotation = [results[tc.VAR_SLOPE], results[tc.VAR_ANGLE], 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = results[tc.VAR_SIGNALS]
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)

        return SumoActor(type_id, vclass, transform, signals, extent, color)

    def get_in_range_actors(self):
        results = traci.vehicle.getContextSubscriptionResults(self.ego_vehicle)
        locations = {}
        self.inrange_actors = set()
        if results:
            for actor in results.keys():
                if results[actor][tc.VAR_VEHICLECLASS]!='bicycle' \
                        and results[actor][tc.VAR_VEHICLECLASS] != 'mortorcycle' \
                        and int(actor)<50:
                    locations[actor] = list(results[actor][tc.VAR_POSITION])
                    self.inrange_actors.add(actor)
        # if self.ego_vehicle in self.inrange_actors:
        #     self.inrange_actors.remove(self.ego_vehicle)
        return locations

    def spawn_actor(self, type_id, color=None, actor_id=None):
        """
        Spawns a new actor.

            :param type_id: vtype to be spawned.
            :param color: color attribute for this specific actor.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        if actor_id is None:
            actor_id = 'carla' + str(self._sequential_id)
            self._sequential_id += 1
            
        try:
            traci.vehicle.add(actor_id, 'carla_route', typeID=type_id)
        except traci.exceptions.TraCIException as error:
            logging.error('Spawn sumo actor failed: %s', error)
            return INVALID_ACTOR_ID

        if color is not None:
            color = color.split(',')
            traci.vehicle.setColor(actor_id, color)

        return actor_id

    @staticmethod
    def destroy_actor(actor_id):
        """
        Destroys the given actor.
        """
        traci.vehicle.remove(actor_id)

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        return self.traffic_light_manager.get_state(landmark_id)

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        self.traffic_light_manager.switch_off()

    def synchronize_vehicle(self, vehicle_id, transform, signals=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param signals: new vehicle signals.
            :return: True if successfully updated. Otherwise, False.
        """
        loc_x, loc_y = transform.location.x, transform.location.y
        yaw = transform.rotation.yaw

        traci.vehicle.moveToXY(vehicle_id, "", 0, loc_x, loc_y, angle=yaw, keepRoute=2)
        if signals is not None:
            traci.vehicle.setSignals(vehicle_id, signals)
        return True

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param tl_id: id of the traffic light to be updated (logic id, link index).
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        self.traffic_light_manager.set_state(landmark_id, state)

    def adjust_vehicle_colors(self):
        """Adjust the colors of the vehicles only when the ego vehicle is alive.
           Red   : ego vehicle
           Green : neighbors in the range of ego vehicle
           Blue  : vehicle in the collective perception network
           Yellow: plain participants
           Color format: RGBA
        """
        if self.ego_vehicle_state==1:
            for vehicle in traci.vehicle.getIDList():
                if vehicle in self.inrange_actors:
                    if vehicle == self.ego_vehicle:
                        traci.vehicle.setColor(vehicle, (255, 0, 0, 255))
                    elif vehicle in self.perception_actors:
                        traci.vehicle.setColor(vehicle, (255, 0, 255, 255))
                    else:
                        traci.vehicle.setColor(vehicle, (0, 255, 0, 255))
                else:
                    traci.vehicle.setColor(vehicle, (255, 255, 0, 255))

    def update_perception_nodes_geo(self, locations, n_samples=5):
        choosen = [self.ego_vehicle]
        not_choosen = list(locations.copy().keys())
        not_choosen.remove(self.ego_vehicle)
        for _ in range(n_samples):
            max_dist = 0
            max_idx = -1
            for i, k in enumerate(not_choosen):
                min_dist = min([np.linalg.norm(np.array(locations[actor]) - np.array(locations[k])) for actor in choosen])
                if max_dist < min_dist:
                    max_dist =  min_dist
                    max_idx = not_choosen[i]
            if max_idx!=-1:
                choosen.append(max_idx)
                not_choosen.remove(max_idx)
        # choosen.remove(self.ego_vehicle)
        self.perception_actors = set(choosen)

    def update_perception_nodes_fps(self, locations, n_samples=5):
        if len(locations) <= 5:
            self.perception_actors = set(list(locations.copy().keys()))
            return
        solution_set = [self.ego_vehicle]
        remaining_points = list(locations.copy().keys())
        remaining_points.remove(self.ego_vehicle)

        def distance(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        for _ in range(n_samples - 1):
            distances = [distance(locations[p], locations[solution_set[0]]) for p in remaining_points]
            for i, p in enumerate(remaining_points):
                for j, s in enumerate(solution_set):
                    distances[i] = min(distances[i], distance(locations[p], locations[s]))
            solution_set.append(remaining_points.pop(distances.index(max(distances))))
        self.perception_actors = set(solution_set)

    def tick(self):
        """
        Tick to sumo simulation.
        """
        traci.simulationStep()
        self.traffic_light_manager.tick()

        # Update data structures for the current frame.
        self.spawned_actors = set(traci.simulation.getDepartedIDList())
        self.destroyed_actors = set(traci.simulation.getArrivedIDList())
        if self.ego_vehicle in self.spawned_actors:
            self.ego_vehicle_state = 1
            self.subscribe_context()
        elif (self.ticks>150 and len(self.inrange_actors)<1) or self.ticks>800:
            self.ego_vehicle_state = 2
        # Update actors that are in the range of ego vehicle and reset the color for all vehicles
        # if ego vehicle is alive, otherwise terminate the simulation:
        #   Red: ego vehicle,
        #   Green: neighbor in range of ego vehicle,
        #   Blue: neighbors which participate in the collective perception
        #   Yellow: normal participants
        if self.ego_vehicle_state==1:
            self.ticks += 1
            perception_nodes_last_step = self.perception_actors.copy()
            locations = self.get_in_range_actors()
            # Sample actors/node from in-range actors for collective perception
            self.update_perception_nodes_fps(locations)
            # Update vehicle colors according to their role in the scene
            self.adjust_vehicle_colors()
            self.sensor_to_spawn = self.perception_actors.difference(perception_nodes_last_step)
            self.sensor_to_stop = perception_nodes_last_step.difference(self.perception_actors)
        elif self.ego_vehicle_state==2:
            self.sensor_to_stop = self.perception_actors

    @staticmethod
    def close():
        """
        Closes traci client.
        """
        traci.close()
