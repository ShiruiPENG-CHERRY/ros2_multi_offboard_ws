#!/usr/bin/env python3
"""
Launches the MPC swarm — 5-drone version (cross topology).

Cross topology (subset of original 9-drone grid):

             3 (north)
             │
   2 (west) ─ 0 (center) ─ 1 (east)
             │
             4 (south)

  - 1 virtual_leader_node
  - 5 mpc_node (drone_id=0..4)
  - 1 arming_node

Leader speed is 0.0 — drones should converge to a static cross at z=-5.
After hover is verified, raise leader speed in steps (0.5 -> 1.0 -> 2.0).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# =====================================================================
# Configuration
# =====================================================================

NUM_DRONES = 5

# Spawn positions in WORLD-NED (x=north, y=east, z=down).
# Only 5 entries — must match NUM_DRONES.
BIRTH_POSITIONS_FLAT = [
    0.0,  0.0, 0.0,   # 0: center
    0.0,  3.0, 0.0,   # 1: east
    0.0, -3.0, 0.0,   # 2: west
    3.0,  0.0, 0.0,   # 3: north
   -3.0,  0.0, 0.0,   # 4: south
]

# Same shape as spawn = formation translates to wherever the leader goes.
FORMATION_OFFSETS_FLAT = BIRTH_POSITIONS_FLAT

# Sparse cross topology — leader is implicit.
NEIGHBOURS_PER_DRONE = {
    0: [1, 2, 3, 4],        # center sees all 4 arms
    1: [0],                 # east sees center
    2: [0],                 # west sees center
    3: [0],                 # north sees center
    4: [0],                 # south sees center
}

COMMON_PARAMS = {
    'num_drones': NUM_DRONES,
    'birth_positions_flat': BIRTH_POSITIONS_FLAT,
    'formation_offsets_flat': FORMATION_OFFSETS_FLAT,

    'target_alt': -5.0,
    'max_speed': 5.0,
    'max_climb': 1.5,
    'max_accel': 5.0,

    'control_hz': 50.0,
    'neighbour_timeout': 1.0,
    'startup_zero_vel_frames': 30,

    # MPC weights — tuned values from this session
    'mpc_horizon': 20,
    'mpc_dt':      0.05,
    'q_pos':       8.0,
    'q_vel':       2.0,
    'r_acc':       0.05,
    'q_pos_terminal_scale': 2.0,
    'd_safe':      1.5,
    'w_collision': 500.0,
    'w_formation': 0.5,
    'acados_build_dir': '/tmp/acados_di_mpc',
}

LEADER_PARAMS = {
    'speed': 1.0,                    # static hover test first
    'altitude': -5.0,
    'publish_hz': 50.0,
    'waypoints_flat': [0.0, 0.0,  0.0, 50.0,  50.0, 50.0,  50.0, 0.0],
}

ARMING_PARAMS = {
    'num_drones': NUM_DRONES,        # 5, must match
    'setup_seconds': 20.0,           # acados compiles faster for 5 drones
    'arm_interval': 0.5,
}


def generate_launch_description():
    nodes = []

    nodes.append(Node(
        package='mpc_control',
        executable='virtual_leader_node',
        name='virtual_leader',
        output='screen',
        parameters=[LEADER_PARAMS],
    ))

    for i in range(NUM_DRONES):
        params = dict(COMMON_PARAMS)
        params['drone_id'] = i
        params['neighbours'] = NEIGHBOURS_PER_DRONE[i]
        nodes.append(Node(
            package='mpc_control',
            executable='mpc_node',
            name=f'mpc_controller_{i}',
            output='screen',
            parameters=[params],
        ))

    nodes.append(Node(
        package='mpc_control',
        executable='arming_node',
        name='arming_node',
        output='screen',
        parameters=[ARMING_PARAMS],
    ))

    return LaunchDescription(nodes)