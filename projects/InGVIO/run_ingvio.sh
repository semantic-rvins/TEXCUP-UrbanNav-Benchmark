#!/bin/bash
# Validate the configuration and inputs using RUN.md before a benchmark replay.
# Headless InGVIO run on the TEX-CUP bag.
# Usage: bash run_ingvio.sh [rate] [bag] [out_bag]
RATE=${1:-1.0}
I="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BAG=${2:-$I/bags/texcup_ingvio.bag}
OUT=${3:-$I/results/ingvio_out.bag}
PORT=11953
source "$I/../GVINS/env.sh"
source "$I/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:$PORT
export ROS_HOSTNAME=localhost
export ROS_HOME="$I/results/ros_home"
mkdir -p "$I/results/logs" "$I/results/ros_home"

# The upstream parser expects a filesystem path inside its YAML.
CONFIG="$I/results/ingvio_mono_texcup.yaml"
python - "$I/config/ingvio_mono_texcup.yaml" "$CONFIG" "$I/config/texcup_port_mono_config.yaml" <<'PY'
import json
from pathlib import Path
import re
import sys
template, output, camera = map(Path, sys.argv[1:])
text, count = re.subn(r'^cam_left_file_path:.*$',
                     'cam_left_file_path: ' + json.dumps(str(camera)),
                     template.read_text(), flags=re.MULTILINE)
assert count == 1
output.write_text(text)
PY

roscore -p "$PORT" > "$I/results/logs/roscore.log" 2>&1 &
ROSCORE_PID=$!
sleep 6

rosbag record -O "$OUT" \
  /ingvio_estimator/pose_ecef /ingvio_estimator/pose_w \
  /ingvio_estimator/pose_spp \
  __name:=ingvio_rec > "$I/results/logs/record.log" 2>&1 &
sleep 4

roslaunch "$I/config/texcup_mono.launch" ingvio_config_path:="$CONFIG" \
  > "$I/results/logs/ingvio.log" 2>&1 &
LAUNCH_PID=$!
sleep 12

rosbag play "$BAG" --rate "$RATE" -d 3 --quiet > "$I/results/logs/play.log" 2>&1
PLAY_RC=$?
echo "rosbag play done rc=$PLAY_RC"
sleep 10

rosnode kill /ingvio_rec > /dev/null 2>&1
sleep 5
kill $LAUNCH_PID > /dev/null 2>&1
sleep 3
kill $ROSCORE_PID > /dev/null 2>&1
sleep 2
pkill -f "port $PORT" 2>/dev/null
echo done
