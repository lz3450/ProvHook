#!/bin/bash

set -e

export MYTURTLESIM_TF2_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1; pwd -P)/.."
myturtlesim_tf2_launch_log_dir="$MYTURTLESIM_TF2_DIR/log"


declare -i pid

# trap "trap - SIGINT SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
function clean_int {
    trap - SIGINT
    echo "INT myturtle ($pid)" >&2
    kill -INT -- -$pid
    exit 0
}
trap clean_int SIGINT

function clean_term {
    trap - SIGTERM
    echo "TERM ($pid)" >&2
    kill -INT -- -$pid
    exit 0
}
trap clean_term SIGTERM

function clean {
    if [ -n $pid ]; then
        echo "EXIT ($pid)" >&2
        kill -INT -- -$pid
        exit 0
    fi
}
trap clean EXIT

function launch {
    local _scale_forward="$1"
    local _scale_rotation="$2"
    local _screenshot_file="$myturtlesim_tf2_launch_log_dir/${_scale_forward/./}_${_scale_rotation/./}/launch.png"

    echo "$_scale_forward $_scale_rotation"
    setsid ./launch_myturtle_tf2.sh "$_scale_forward" "$_scale_rotation" > /dev/null &
    # Store the PID of the last background process
    pid=$!
    echo "PID: $pid"

    sleep 25

    gnome-screenshot -w -f "$_screenshot_file"
    echo "Screenshot saved to $_screenshot_file"

    # Kill the specific process using its PID
    kill -INT -- -$pid
}

if [ "$#" -eq 0 ]; then
    declare -a scales
    for ((i=5; i<=80; i+=5)); do
        result=$(bc <<< "scale=1; $i/10")
        # Check if the result begins with a dot and prepend '0' if necessary.
        if [[ $result == .* ]]; then
            result="0$result"
        fi
        scales[i]=$result
    done

    for scale_forward in "${scales[@]}"; do
        for scale_rotation in "${scales[@]}"; do
            launch $scale_forward $scale_rotation
        done
    done
elif [ "$#" -eq 2 ]; then
    launch "$1" "$2"
else
    echo "Usage: $0 [ <scale_forward> <scale_rotation> ]"
    exit 1
fi
