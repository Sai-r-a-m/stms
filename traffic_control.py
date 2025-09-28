import os
import sys
import traci
import random
import numpy as np

sumo_binary = os.path.join("sumo-gui.exe")
config_file = "simulation.sumocfg"

lanes = ["N_C_0", "S_C_0", "E_C_0", "W_C_0", "C_N_0", "C_S_0", "C_E_0", "C_W_0"]

TLS_ID = "C"

vehicle_types = ["car", "truck", "bike"]
routes_dict = {
    "north": ["N_left", "N_straight", "N_right"],
    "south": ["S_left", "S_straight", "S_right"],
    "east":  ["E_left", "E_straight", "E_right"],
    "west":  ["W_left", "W_straight", "W_right"]
}


vehicle_counter = 0

def get_state():
    state = []
    for lane in lanes:
        count = traci.lane.getLastStepVehicleNumber(lane)
        wait = traci.lane.getWaitingTime(lane)
        state.append(count)
        state.append(wait)
    return np.array(state)

def get_reward():
    total_wait = sum(traci.lane.getWaitingTime(lane) for lane in lanes)
    return -total_wait

def take_action(action):
    traci.trafficlight.setPhase(TLS_ID, action)

def run():
    global vehicle_counter

    traci.start([sumo_binary, "-c", config_file])
    step = 0
    current_phase = 0
    phase_duration = 30

    while step < 10000:
        traci.simulationStep()

        if step % 1 == 0:
            incoming = random.choice(["north", "south", "east", "west"])
            route = random.choice(routes_dict[incoming])
            vtype = random.choice(vehicle_types)
            veh_id = f"veh{vehicle_counter}"

            if veh_id not in traci.vehicle.getIDList():
                traci.vehicle.add(veh_id, routeID=route, typeID=vtype)
                vehicle_counter += 1


        if step % phase_duration == 0:
            current_phase = (current_phase + 1) % 2
            take_action(current_phase)

        state = get_state()
        reward = get_reward()

        if step % 100 == 0:
            print(f"Step {step}: Reward = {reward}, Active Vehicles = {len(traci.vehicle.getIDList())}")

        step += 1

    traci.close()

if __name__ == "__main__":
    run()