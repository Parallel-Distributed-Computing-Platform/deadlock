from __future__ import absolute_import
from __future__ import print_function
import csv
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


def make_deadlock_vehicle(num):
    color_dic = {
        1: (255, 0, 0),
        2: (0, 255, 0),
        3: (0, 0, 255),
        4: (0, 255, 255),
        5: (255, 0, 255),
        6: (255, 255, 0)
    }
    for i in range(1, num):
        traci.route.add(f"deadlock{i}_1", ["gneE16", "gneE42"])
        traci.route.add(f"deadlock{i}_2", ["gneE27", "gneE41"])
        traci.route.add(f"deadlock{i}_3", ["-gneE17", "gneE43"])
        traci.route.add(f"deadlock{i}_4", ["-gneE42", "-gneE16"])
        traci.route.add(f"deadlock{i}_5", ["-gneE41", "-gneE27"])
        traci.route.add(f"deadlock{i}_6", ["-gneE43", "gneE17"])
        for k in range(1, 7):
            traci.vehicle.addLegacy(f"deadlock{i}_{k}", f"deadlock{i}_{k}", 1)
            traci.vehicle.setSpeedMode(f"deadlock{i}_{k}", 55)
            traci.vehicle.setColor(f"deadlock{i}_{k}", color_dic[k])


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
            traci.vehicle.setSpeed(vehicle, SPEED)
        else:
            traci.vehicle.setSpeed(vehicle, 0)


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

SPEED = 5
DISTANCE = SPEED * 10
net = sumolib.net.readNet('data/deadlock.net.xml')

PRIORITY = {"gneJ12": "PRIORITY"}


def simulation(num, is_gui):

    init(is_gui)
    make_deadlock_vehicle(num)
    while traci.simulation.getMinExpectedNumber() > 0:
        solve_deadlock("gneJ12", 0, {})
        traci.simulationStep()
    traci.close()


num_of_vehicles = int(input("Num of vehicles :"))
simulation(num_of_vehicles, True)
