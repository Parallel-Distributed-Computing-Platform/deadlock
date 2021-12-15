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
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
from sumolib import checkBinary
import traci
import traci.constants as tc
import sumolib

SPEED = 5
depart_arr = [
    "-gneE38", "-gneE39", "-gneE40", "-gneE41", "-gneE42", "-gneE43",
    "-gneE44", "-gneE45", "-gneE46", "-gneE47", "-gneE48", "-gneE49"
]
arrival_arr = [
    "gneE38", "gneE39", "gneE40", "gneE41", "gneE42", "gneE43", "gneE44",
    "gneE45", "gneE46", "gneE47", "gneE48", "gneE49"
]
#Init


def init(is_gui):
    if is_gui:
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    else:
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo')

    sumoCmd = [sumoBinary, "-c", "data/deadlock.sumocfg"]
    tree = ET.parse('data/allRoute.rou.xml')
    root = tree.getroot()
    traci.start(sumoCmd)


def detect_gridlock(vehID, next_lane):
    try:
        locked_veh = traci.lane.getLastStepVehicleIDs(next_lane)[-1]
        locked_veh_speed = traci.vehicle.getSpeed(locked_veh)
        if locked_veh_speed == 0:
            return vehID, locked_veh
    except:
        pass


def control_vehicle(vehID, net):
    try:
        lanes = traci.vehicle.getBestLanes(vehID)[0][5]
        current_edge = traci.lane.getEdgeID(lanes[0])
        nextNodeID = net.getEdge(current_edge).getToNode().getID()
        vehicle_pos = traci.vehicle.getPosition(vehID)
        junction_pos = traci.junction.getPosition(nextNodeID)
        junction_vehicle_distance = traci.simulation.getDistance2D(
            vehicle_pos[0], vehicle_pos[1], junction_pos[0], junction_pos[1])

        if 1 < junction_vehicle_distance < 35:
            judge_num = (traci.lane.getLength(lanes[1]) *
                         (1 - traci.lane.getLastStepOccupancy(lanes[1])))

            if judge_num < 50:
                traci.vehicle.setSpeed(vehID, 0)
            else:
                traci.vehicle.setSpeed(vehID, SPEED)

        else:
            traci.vehicle.setSpeed(vehID, SPEED)

    except:
        pass


#Start
net = sumolib.net.readNet('data/deadlock.net.xml')
#Gridlock error
color_dic = {
    1: (255, 0, 0),
    2: (0, 255, 0),
    3: (0, 0, 255),
    4: (255, 255, 255)
}


def simulation(num_of_vehicles, with_program):
    init(True)
    for i in range(1, num_of_vehicles):
        traci.route.add(
            f"{i}_1",
            ["gneE9", "-gneE1", "-gneE0", "-gneE3", "-gneE26", "gneE38"])
        traci.route.add(
            f"{i}_2",
            ["gneE15", "-gneE3", "-gneE2", "-gneE1", "-gneE24", "gneE45"])
        traci.route.add(
            f"{i}_3",
            ["gneE6", "-gneE2", "-gneE1", "-gneE0", "gneE27", "gneE41"])
        traci.route.add(
            f"{i}_4",
            ["gneE12", "-gneE0", "-gneE3", "-gneE2", "-gneE25", "gneE49"])
        for k in range(1, 5):
            traci.vehicle.addLegacy(f"{k}_{i}", f"{i}_{k}", 0)
            traci.vehicle.setMaxSpeed(f"{k}_{i}", SPEED)
            traci.vehicle.setColor(f"{k}_{i}", color_dic[k])
    while traci.simulation.getMinExpectedNumber() > 0:
        if with_program:
            locked_vehicles = {}
            leaders = []
            gridlock = []
            control_arr = []
            current_time = traci.simulation.getTime()
            all_vehicles = traci.vehicle.getIDList()

            for vehicle in all_vehicles:
                leader = (traci.vehicle.getLeader(vehicle))
                vehicle_lane = traci.vehicle.getLaneID(vehicle)
                try:
                    if vehicle_lane[0] != ":":
                        if leader == None:
                            leaders.append(vehicle)
                        else:
                            leader_lane = traci.vehicle.getLaneID(leader[0])
                            if vehicle_lane != leader_lane:
                                leaders.append(vehicle)
                except:
                    pass
            for leader in leaders:
                leader_lanes = traci.vehicle.getBestLanes(leader)[0][5]
                control_arr.append(leader)

                if len(leader_lanes) > 1:
                    locked_veh = detect_gridlock(leader, leader_lanes[1])
                    if locked_veh:
                        locked_vehicles[locked_veh[0]] = locked_veh[1]

            for init_veh in (locked_vehicles):
                if len(gridlock) > 0:
                    if init_veh not in gridlock[0]:
                        temp = []
                        index = init_veh
                        for k in range(15):
                            try:
                                next_vehicle = locked_vehicles[index]
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
                    for k in range(15):
                        try:
                            next_vehicle = locked_vehicles[index]
                            temp.append(next_vehicle)
                            index = next_vehicle
                            if next_vehicle == init_veh:
                                gridlock.append(temp)
                                break

                        except:
                            break

            for v in control_arr:
                control_vehicle(v, net)
            for vehicles in gridlock:
                for v in vehicles:
                    traci.vehicle.setSpeed(v, SPEED)
        traci.simulationStep()
        if traci.simulation.getTime() == 5000:
            traci.close()
    traci.close()


num_of_vehicles = int(input("Num of vehicles :"))
with_program = int(input("With control program (0=false, 1=true):"))
simulation(num_of_vehicles, bool(with_program))
