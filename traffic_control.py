import os
import traci
import random
import numpy as np

# SUMO configuration
sumo_binary = "sumo-gui.exe"  # Change to "sumo" if you want no GUI
config_file = "simulation.sumocfg"

# Traffic light ID
TLS_ID = "C"

# Vehicle types
vehicle_types = ["car", "truck", "bike"]

# Routes for each incoming road
routes_dict = {
    "north": ["N_left", "N_straight", "N_right"],
    "south": ["S_left", "S_straight", "S_right"],
    "east":  ["E_left", "E_straight", "E_right"],
    "west":  ["W_left", "W_straight", "W_right"]
}

# Lanes for state measurement
lanes = ["N_C_0", "S_C_0", "E_C_0", "W_C_0", "C_N_0", "C_S_0", "C_E_0", "C_W_0"]

vehicle_counter = 0

def get_state():
    """Return the number of vehicles and waiting time per lane."""
    state = []
    for lane in lanes:
        count = traci.lane.getLastStepVehicleNumber(lane)
        wait = traci.lane.getWaitingTime(lane)
        state.append(count)
        state.append(wait)
    return np.array(state)

def get_reward():
    """Reward function: negative of total waiting time."""
    total_wait = sum(traci.lane.getWaitingTime(lane) for lane in lanes)
    return -total_wait

def take_action(phase):
    """Set traffic light phase."""
    # Each phase allows one road to go
    traci.trafficlight.setPhase(TLS_ID, phase)

def run():
    global vehicle_counter

    traci.start([sumo_binary, "-c", config_file])
    step = 0
    current_phase = 0
    phase_duration = 172  # steps per traffic light phase

    while step < 10000:
        traci.simulationStep()

        # Add a random vehicle every step
        if step%3==0:
            incoming = random.choice(["north", "south", "east", "west"])
            route = random.choice(routes_dict[incoming])
            vtype = random.choice(vehicle_types)
            veh_id = f"veh{vehicle_counter}"

            if veh_id not in traci.vehicle.getIDList():
                traci.vehicle.add(veh_id, routeID=route, typeID=vtype)
                vehicle_counter += 1

        # Change traffic light every phase_duration steps (2 phases only)
        if step % phase_duration == 0:
            current_phase = (current_phase + 1) % 2
            take_action(current_phase)

        # Get state and reward
        state = get_state()
        reward = get_reward()

        # Debug output
        if step % 100 == 0:
            print(f"Step {step}: Reward = {reward}, Active Vehicles = {len(traci.vehicle.getIDList())}")

        step += 1

    traci.close()


if __name__ == "__main__":
    run()

