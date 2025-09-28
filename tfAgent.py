import os
import traci
import random
import numpy as np
from collections import deque
import tensorflow as tf
from tensorflow.keras import layers, models, optimizers
import matplotlib.pyplot as plt
import pickle

# ------------------- SUMO CONFIG -------------------
sumo_binary = "sumo-gui.exe"
config_file = "simulation.sumocfg"
TLS_ID = "C"

vehicle_types = ["car", "truck", "bike"]
routes_dict = {
    "north": ["N_left", "N_straight", "N_right"],
    "south": ["S_left", "S_straight", "S_right"],
    "east":  ["E_left", "E_straight", "E_right"],
    "west":  ["W_left", "W_straight", "W_right"]
}

lanes = ["N_C_0", "S_C_0", "E_C_0", "W_C_0", "C_N_0", "C_S_0", "C_E_0", "C_W_0"]
vehicle_counter = 0

# ------------------- DQN PARAMETERS -------------------
STATE_SIZE = len(lanes) * 2  # count + waiting per lane
ACTION_SIZE = 4
MIN_GREEN = 10

GAMMA = 0.95
LEARNING_RATE = 0.001
MEMORY_SIZE = 50000
BATCH_SIZE = 64
EPSILON = 1.0
EPSILON_DECAY = 0.99995
EPSILON_MIN = 0.05

REPLAY_MEMORY = deque(maxlen=MEMORY_SIZE)

# Traffic light states
actions = [
    "GGGGggrrrrrrrrrrrrrrrrrr",  # N-S green
    "rrrrrrGGGGggrrrrrrrrrrrr",  # E-W green
    "rrrrrrrrrrrrGGGGggrrrrrr",  # N-C green
    "rrrrrrrrrrrrrrrrrrGGGGgg",  # C-S green
]

# ------------------- DQN MODEL -------------------
def build_model(state_size, action_size):
    model = models.Sequential()
    model.add(layers.Dense(128, input_dim=state_size, activation='relu'))
    model.add(layers.Dense(128, activation='relu'))
    model.add(layers.Dense(action_size, activation='linear'))
    model.compile(loss='mse', optimizer=optimizers.Adam(learning_rate=LEARNING_RATE))
    return model

model = build_model(STATE_SIZE, ACTION_SIZE)
target_model = build_model(STATE_SIZE, ACTION_SIZE)
target_model.set_weights(model.get_weights())

# ------------------- HELPER FUNCTIONS -------------------
def get_state():
    state = []
    for lane in lanes:
        count = traci.lane.getLastStepVehicleNumber(lane)
        wait = traci.lane.getWaitingTime(lane)
        state.append(count)
        state.append(wait)
    return np.array(state)

def get_reward(prev_total_wait, teleport_count):
    curr_total_wait = sum(traci.lane.getWaitingTime(lane) for lane in lanes)
    delta_wait = prev_total_wait - curr_total_wait
    reward = delta_wait - teleport_count * 50  # penalize teleporting heavily
    return reward, curr_total_wait

def take_action(action_idx):
    state_str = actions[action_idx]
    traci.trafficlight.setRedYellowGreenState(TLS_ID, state_str)

def choose_action(state, epsilon):
    if np.random.rand() < epsilon:
        return np.random.randint(ACTION_SIZE)
    q_values = model.predict(state[np.newaxis], verbose=0)
    return np.argmax(q_values[0])

def replay():
    if len(REPLAY_MEMORY) < BATCH_SIZE:
        return
    batch = random.sample(REPLAY_MEMORY, BATCH_SIZE)
    states = np.array([b[0] for b in batch])
    actions_idx = np.array([b[1] for b in batch])
    rewards = np.array([b[2] for b in batch])
    next_states = np.array([b[3] for b in batch])
    dones = np.array([b[4] for b in batch])

    target = model.predict(states, verbose=0)
    target_next = target_model.predict(next_states, verbose=0)

    for i in range(BATCH_SIZE):
        if dones[i]:
            target[i][actions_idx[i]] = rewards[i]
        else:
            target[i][actions_idx[i]] = rewards[i] + GAMMA * np.max(target_next[i])

    model.fit(states, target, epochs=1, verbose=0)

def update_target():
    target_model.set_weights(model.get_weights())

def save_dqn(filename="dqn_model.pkl"):
    with open(filename, "wb") as f:
        pickle.dump(model.get_weights(), f)
    print(f"DQN model weights saved to {filename}")

def load_dqn(filename="dqn_model.pkl"):
    global model, target_model
    with open(filename, "rb") as f:
        weights = pickle.load(f)
        model.set_weights(weights)
        target_model.set_weights(weights)
    print(f"DQN model weights loaded from {filename}")

# ------------------- MAIN TRAINING LOOP -------------------
def run():
    global vehicle_counter, EPSILON
    traci.start([sumo_binary, "-c", config_file])

    step = 0
    phase_timer = 0
    current_phase = 0
    prev_total_wait = 0
    waiting_history = []

    while step < 10000:
        traci.simulationStep()

        # Vehicle generation
        if step % 3 == 0:
            incoming = random.choice(["north", "south", "east", "west"])
            route = random.choice(routes_dict[incoming])
            vtype = random.choice(vehicle_types)
            veh_id = f"veh{vehicle_counter}"
            if veh_id not in traci.vehicle.getIDList():
                try:
                    traci.vehicle.add(veh_id, routeID=route, typeID=vtype)
                    vehicle_counter += 1
                except:
                    pass

        # Count teleported vehicles
        teleport_count = 0
        for veh in traci.vehicle.getIDList():
            if traci.vehicle.getWaitingTime(veh) > 1000:
                teleport_count += 1

        # Take RL action only if minimum green duration passed
        if phase_timer >= MIN_GREEN:
            state = get_state()
            action_idx = choose_action(state, EPSILON)
            take_action(action_idx)

            reward, prev_total_wait = get_reward(prev_total_wait, teleport_count)
            next_state = get_state()
            done = False

            REPLAY_MEMORY.append((state, action_idx, reward, next_state, done))
            replay()
            update_target()

            current_phase = action_idx
            phase_timer = 0

        phase_timer += 1

        # Record total waiting time for graph
        total_wait = sum(traci.lane.getWaitingTime(lane) for lane in lanes)
        waiting_history.append(total_wait)

        # Decay epsilon
        if EPSILON > EPSILON_MIN:
            EPSILON *= EPSILON_DECAY

        # Logging
        if step % 100 == 0:
            print(f"Step {step}, Active Vehicles: {len(traci.vehicle.getIDList())}, Total Wait: {total_wait:.2f}, Epsilon: {EPSILON:.3f}")

        step += 1

    traci.close()

    # Save DQN model
    save_dqn()

    # Plot waiting time
    plt.figure(figsize=(15,5))
    plt.plot(waiting_history, color='blue', label="Total Waiting Time")
    plt.title("Total Waiting Time vs Simulation Step")
    plt.xlabel("Simulation Step")
    plt.ylabel("Total Waiting Time")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run()
