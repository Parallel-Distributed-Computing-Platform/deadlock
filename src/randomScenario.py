from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import random
import numpy as np
import xml.etree.ElementTree as ET

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    print(os.environ['SUMO_HOME'])
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
from sumolib import checkBinary
import traci
import traci.constants as tc
import sumolib


def make_vehicle(vehID, routeID, depart_time):
    traci.vehicle.addLegacy(vehID, routeID, depart=depart_time)
    traci.vehicle.setSpeed(vehID, SPEED)
    traci.vehicle.setMaxSpeed(vehID, SPEED)


def make_random_route(num):
    ok = True
    while ok:
        depart = random.choice(DEPART)
        arrive = random.choice(ARRIVAL)
        try:
            traci.route.add(f"random_route_{num}", [depart, arrive])
            ok = False
        except:
            pass
    return f"random_route_{num}"


#Init
def get_distacne(vehID, net):
    try:
        current_edge = traci.vehicle.getRoadID(vehID)
        nextNodeID = net.getEdge(current_edge).getToNode().getID()
        vehicle_pos = traci.vehicle.getPosition(vehID)
        junction_pos = traci.junction.getPosition(nextNodeID)
        junction_vehicle_distance = traci.simulation.getDistance2D(
            vehicle_pos[0], vehicle_pos[1], junction_pos[0], junction_pos[1])

        return junction_vehicle_distance

    except:
        pass


def control_junction(control_obj, index, nodeID, locked_vehicle):

    for i, edge in enumerate(control_obj):
        vehicle = control_obj[edge]

        if PRIORITY[nodeID] not in control_obj:
            if i == 0:
                PRIORITY[nodeID] = edge

        if PRIORITY[nodeID] == edge:
            distance = get_distacne(vehicle, net)
            traci.vehicle.setColor(vehicle, (255, 0, 0))
            traci.vehicle.setSpeed(vehicle, SPEED)
            leader = traci.vehicle.getLeader(vehicle, DISTANCE)
            if distance < DISTANCE:
                try:
                    lanes = traci.vehicle.getBestLanes(vehicle)[0][5]
                    judge_num = traci.lane.getLastStepOccupancy(
                        lanes[1]) * traci.lane.getLength(lanes[1])
                    if traci.lane.getLastStepOccupancy(lanes[1]) > 0.3:

                        detect_gridlock(vehicle, lanes, locked_vehicle)
                        traci.vehicle.setColor(vehicle, (0, 0, 255))
                        traci.vehicle.setSpeed(vehicle, 0)
                    else:
                        try:
                            traci.vehicle.setSpeed(leader[0], SPEED)
                        except:
                            pass
                        traci.vehicle.setSpeed(vehicle, SPEED)

                except:
                    pass
        else:
            traci.vehicle.setSpeed(vehicle, 0)


def is_priority(lane):
    if traci.lane.getEdgeID(lane) not in PRIORITY.values():
        return False
    return True


def detect_gridlock(vehicle, lanes, locked_vehicle):
    if is_priority(lanes[1]):
        next_vehicle = traci.lane.getLastStepVehicleIDs(lanes[1])[-1]
        next_lane = traci.vehicle.getBestLanes(next_vehicle)[0][5][1]
        locked_vehicle[vehicle] = next_vehicle
        return
    else:
        next_edge = traci.lane.getEdgeID(lanes[1])
        nextNodeID = net.getEdge(next_edge).getToNode().getID()
        next_vehicle = traci.edge.getLastStepVehicleIDs(
            PRIORITY[nextNodeID])[-1]

        locked_vehicle[vehicle] = next_vehicle
        PRIORITY[nextNodeID] = next_edge

    return True


def solve_deadlock(nodeID, index, locked_vehicle):
    control_obj = {}
    junction_edges = []
    for edge in net.getNode(nodeID).getIncoming():
        junction_edges.append(edge.getID())

    for edge in junction_edges:
        try:
            v = traci.edge.getLastStepVehicleIDs(edge)[-1]
            lane = traci.vehicle.getLaneID(v)
            distance = get_distacne(v, net)
            if distance < DISTANCE:
                control_obj[edge] = v
        except:
            pass
    control_junction(control_obj, index, nodeID, locked_vehicle)


def init(is_gui):
    if is_gui:
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    else:
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo')
    sumoCmd = [sumoBinary, "-c", "data/deadlock.sumocfg"]
    traci.start(sumoCmd)


#Start

DEPART = [
    "-gneE38", "-gneE39", "-gneE40", "-gneE41", "-gneE42", "-gneE43",
    "-gneE44", "-gneE45", "-gneE46", "-gneE47", "-gneE48", "-gneE49"
]
ARRIVAL = [
    "gneE38", "gneE39", "gneE40", "gneE41", "gneE42", "gneE43", "gneE44",
    "gneE45", "gneE46", "gneE47", "gneE48", "gneE49"
]
SPEED = 5
DISTANCE = SPEED * 10
net = sumolib.net.readNet('data/deadlock.net.xml')

NODES = []
PRIORITY = {}
PRIORITY_ARR = []


def simulation(num, with_program):
    for node in net.getNodes():
        NODES.append(node.getID())
        PRIORITY[node.getID()] = "PRIORITY"
    init(True)
    teleportNum = 0
    departedNum = 0
    arrivedNum = 0
    for i in range(num):
        make_vehicle(f"vehicle_{i}", make_random_route(i), 0)
    while traci.simulation.getMinExpectedNumber() > 0:
        if with_program:
            locked_vehicle = {}
            collisions_arr = traci.simulation.getCollisions()
            if collisions_arr:
                print(traci.simulation.getCollisions())
            for index, node in enumerate(NODES):
                solve_deadlock(node, index, locked_vehicle)
            del_arr = []
            for vehicle_key in locked_vehicle.keys():
                if vehicle_key not in locked_vehicle.values():
                    del_arr.append(vehicle_key)
            for key in del_arr:
                del locked_vehicle[key]

            #GRIDLOCK
            gridlock = []
            for init_veh in (locked_vehicle):
                if len(gridlock) > 0:
                    if init_veh not in gridlock[0]:
                        temp = []
                        index = init_veh
                        for k in range(6):
                            try:
                                next_vehicle = locked_vehicle[index]
                                temp.append(next_vehicle)
                                index = next_vehicle
                                if next_vehicle == init_veh:
                                    gridlock.append(temp)
                                    break

                            except:
                                break
                elif len(gridlock) == 0:
                    temp = []
                    index = init_veh
                    for k in range(6):
                        try:
                            next_vehicle = locked_vehicle[index]
                            temp.append(next_vehicle)
                            index = next_vehicle
                            if next_vehicle == init_veh:
                                gridlock.append(temp)
                                break

                        except:
                            break

            for vehicles in gridlock:
                for v in vehicles:
                    traci.vehicle.setSpeed(v, SPEED)
        traci.simulationStep()
        if traci.simulation.getTime() == 3600:
            traci.close()
    traci.close()


num_of_vehicles = int(input("Num of vehicles :"))
with_program = int(input("With control program (0=false, 1=true):"))
simulation(num_of_vehicles, with_program)
