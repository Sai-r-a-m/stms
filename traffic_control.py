import traci
import sumolib
import random
import time

SUMO_BINARY = "sumo-gui"
SUMO_CONFIG = "simulation.sumocfg"
# incoming_lanes = [
#     "N_C",  
#     "S_C",  
#     "E_C",  
#     "W_C"   
# ]

TLS_ID = "C"

lanes = ["N_C_0", "N_C_1", "S_C_0", "S_C_1",
         "C_S_0", "C_S_1", "C_N_0", "C_N_1",
         "E_C_0", "E_C_1", "W_C_0", "W_C_1",
         "C_W_0", "C_W_1", "C_E_0", "C_E_1"]

routes = ["north_south", "south_north", "east_west", "west_east",
          "north_east", "north_west", "south_east", "south_west",
          "east_north", "east_south", "west_north", "west_south"]

# def get_state():
#     vehicle_count = []
#     waiting_time = []

#     for lane in incoming_lanes:
#         count = traci.lane.getLastStepVehicleNumber(lane)
#         wait = traci.lane.getWaitingTime(lane)
#         vehicle_count.append(count)
#         waiting_time.append(wait)

#     current_phase = traci.trafficlight.getPhase("C") 

#     num_phases = traci.trafficlight.getPhaseNumber("C")
#     phase_one_hot = [0] * num_phases
#     phase_one_hot[current_phase] = 1

#     state_vector = vehicle_count + waiting_time + phase_one_hot

#     return state_vector


vehicle_types = ["car", "truck", "bike"]

traci.start([SUMO_BINARY, "-c", SUMO_CONFIG])
step = 0
vehicle_counter = 1000

phases = ["GGrrGGrrGGrrGGrr", "rrGGrrGGrrGGrrGG"] 
current_phase = 0
phase_duration = 10000
phase_timer = 0

waiting_times = []

try:
    while step < 10000:
        traci.simulationStep()
        step += 1

        # /state = get_state()

        lane_vehicles = [traci.lane.getLastStepVehicleNumber(l) for l in lanes]
        total_waiting = sum(traci.lane.getWaitingTime(l) for l in lanes)
        waiting_times.append(total_waiting)

        phase_timer += 1
        if phase_timer >= phase_duration:
            num_phases = traci.trafficlight.getPhaseNumber(TLS_ID)
            current_phase = (current_phase + 1) % num_phases
            traci.trafficlight.setPhase(TLS_ID, current_phase)
            phase_timer = 0

        # if step % 50 == 0:
        #     print(f"Step {step} | State: {state}")
        if step % 2 == 0:
            route = random.choice(routes)
            vtype = random.choice(vehicle_types)
            traci.vehicle.add(f"veh{vehicle_counter}", routeID=route, typeID=vtype, depart=step)
            vehicle_counter += 1

        if step % 50 == 0:
            print(f"Step {step} | Total waiting time: {total_waiting:.2f} | Vehicles on network: {traci.simulation.getMinExpectedNumber()}")

finally:
    traci.close()
