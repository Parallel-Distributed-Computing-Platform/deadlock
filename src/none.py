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


def make_vehicle(vehID, routeID, depart_time, speedmode):
    traci.vehicle.addLegacy(vehID, routeID, depart=depart_time)
    traci.vehicle.setSpeed(vehID, SPEED)
    traci.vehicle.setMaxSpeed(vehID, SPEED)
    if speedmode:
        traci.vehicle.setSpeedMode(vehID, 7)


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


def init(is_gui):
    if is_gui:
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    else:
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo')

    sumoCmd = [sumoBinary, "-c", "data/trafficLightFixed.sumocfg"]
    tree = ET.parse('data/allRoute.rou.xml')
    root = tree.getroot()
    traci.start(sumoCmd)


#Start

DEPART = [
    "-gneE40", "-gneE41", "-gneE42", "-gneE43", "-gneE44", "-gneE45",
    "-gneE46", "-gneE47", "-gneE48", "-gneE49", "-gneE50", "-gneE51"
]
ARRIVAL = [
    "gneE40", "gneE41", "gneE42", "gneE43", "gneE44", "gneE45", "gneE46",
    "gneE47", "gneE48", "gneE49", "gneE50", "gneE51"
]
SPEED = 5
DISTANCE = SPEED * 10


def simulation(num, speedmode):
    init(True)
    for i in range(num):
        make_vehicle(f"vehicle_{i}", make_random_route(i), 0, speedmode)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if traci.simulation.getTime() == 5600:
            traci.close()
    traci.close()


num_of_vehicles = int(input("Num of vehicles :"))
speedmode = int(input("Ignore traffic light (0=false, 1=true):"))
simulation(num_of_vehicles, bool(speedmode))
