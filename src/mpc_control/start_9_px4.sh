#!/bin/bash
# Start 9 PX4 SITL instances in standalone Gazebo Garden mode.
#
# Usage:
#   1. First start Gazebo manually in another terminal:
#        gz sim -r -s ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
#   2. Wait until Gazebo is ready
#   3. In another terminal, run this script:
#        bash start_9_px4.sh
#   4. Each PX4 instance will spawn into its own gnome-terminal tab.
#      You can also adapt this for tmux if you prefer.
#
# Note on PX4_GZ_MODEL_POSE format:
#   "x,y" where x is EAST, y is NORTH (Gazebo ENU). So drone 1 at NED (0, 3, 0)
#   means North=0, East=3, which in Gazebo ENU is x=3, y=0 -> "3,0".

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4_DIR=$PX4_DIR does not exist. Set PX4_DIR env var."
    exit 1
fi

cd "$PX4_DIR"

# Drone configurations: id, gz_pose (east,north)
# Birth positions in WORLD NED (north, east, down):
#   0: (0, 0)        -> ENU (0, 0)    -> "0,0"
#   1: (0, 3)        -> ENU (3, 0)    -> "3,0"
#   2: (0, -3)       -> ENU (-3, 0)   -> "-3,0"
#   3: (3, 0)        -> ENU (0, 3)    -> "0,3"
#   4: (3, 3)        -> ENU (3, 3)    -> "3,3"
#   5: (3, -3)       -> ENU (-3, 3)   -> "-3,3"
#   6: (-3, 0)       -> ENU (0, -3)   -> "0,-3"
#   7: (-3, 3)       -> ENU (3, -3)   -> "3,-3"
#   8: (-3, -3)      -> ENU (-3, -3)  -> "-3,-3"

declare -a POSES=(
    "0,0"
    "3,0"
    "-3,0"
    "0,3"
    "3,3"
    "-3,3"
    "0,-3"
    "3,-3"
    "-3,-3"
)

START_DELAY="${START_DELAY:-30}"  # seconds between successive instances

for i in $(seq 0 8); do
    POSE="${POSES[$i]}"
    echo "Starting drone $i with PX4_GZ_MODEL_POSE=$POSE ..."

    # Launch in a new gnome-terminal tab. If gnome-terminal is unavailable,
    # change to xterm or run sequentially in background with logs to files.
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab --title="px4_$i" -- bash -c "
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL_POSE='$POSE'
            export PX4_SIM_MODEL=gz_x500
            cd '$PX4_DIR'
            ./build/px4_sitl_default/bin/px4 -i $i;
            exec bash
        "
    else
        # Fallback: background with logfiles
        LOG_DIR="$HOME/px4_logs"
        mkdir -p "$LOG_DIR"
        (
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL_POSE="$POSE"
            export PX4_SIM_MODEL=gz_x500
            cd "$PX4_DIR"
            ./build/px4_sitl_default/bin/px4 -i $i > "$LOG_DIR/px4_$i.log" 2>&1
        ) &
        echo "  -> logging to $LOG_DIR/px4_$i.log (PID $!)"
    fi

    # Wait between instances to give PX4 + Gazebo time to settle
    if [ $i -lt 8 ]; then
        echo "  waiting ${START_DELAY}s before next instance..."
        sleep "$START_DELAY"
    fi
done

echo ""
echo "All 9 PX4 instances launched. Now in separate terminals, run:"
echo "  Terminal A:  MicroXRCEAgent udp4 -p 8888"
echo "  Terminal B:  cd ~/ros2_multi_offboard_ws && source install/setup.bash"
echo "               ros2 launch flocking_swarm swarm_launch.py"
