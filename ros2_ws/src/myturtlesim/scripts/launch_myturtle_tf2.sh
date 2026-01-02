#!/bin/bash

set -e
# set -x

export MYTURTLESIM_TF2_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1; pwd -P)/.."
FLOAT_RE='^[0-9]+([.][0-9]+)?$'

scale_forward=0.5
scale_rotation=1.0
log_dir="$MYTURTLESIM_TF2_DIR/log/${scale_forward/./}_${scale_rotation/./}"

# trap "trap - SIGINT SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
# function clean {
#     trap - SIGINT SIGTERM
#     echo "Terminating myturtle ($scale_forward $scale_forward)" >&2
#     kill -- -$$
# }
# trap clean SIGINT SIGTERM EXIT

# Check if two arguments are provided
if [ "$#" -eq 2 ]; then
    # Validate that both arguments are float numbers
    if ! [[ $1 =~ $FLOAT_RE ]] || ! [[ $2 =~ $FLOAT_RE ]]; then
        echo "Error: $FLOAT_REBoth arguments must be positive floating-point numbers."
        exit 2
    fi
    scale_forward=$1
    scale_rotation=$2
elif [ "$#" != 0 ]; then
    echo "Usage: $0 <scale_forward> <scale_rotation>"
    exit 1
fi

if [ -z $ROS_DISTRO ]; then
    source ~/Projects/ros2_ws/ros2_setup.sh
fi

if [ ! -d "$log_dir" ]; then
    mkdir -p "$log_dir"
fi

# log_file="$MYTURTLESIM_TF2_DIR/log/myturtlesim_tf2_launch_$(date '+%y%m%d_%H%M%S').log"
# ln -srf "$log_file" "$MYTURTLESIM_TF2_DIR/log/myturtlesim_tf2_launch.log"
log_file="$log_dir/launch.log"

ros2 launch myturtlesim myturtlesim_tf2_launch.py scale_forward:=$scale_forward scale_rotation:=$scale_rotation | tee "$log_file"
