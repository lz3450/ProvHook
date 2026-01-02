if [ -n "$BASH_VERSION" ]; then
    export ROS2_WS="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1; pwd -P)"
    ROS2_SHELL="bash"
elif [ -n "$ZSH_VERSION" ]; then
    export ROS2_WS="$(cd -- "$(dirname "${(%):-%x}")" > /dev/null 2>&1; pwd -P)"
    ROS2_SHELL="zsh"
else
    echo "Unsupported shell"
fi
echo "ROS2_WS=$ROS2_WS"

# export ROS_DOMAIN_ID=77
# echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
# export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
# export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
# echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

ROS2_SETUP="$ROS2_WS/install/local_setup.$ROS2_SHELL"
if [[ -f "$ROS2_SETUP" ]]; then
    echo "ros2 ($ROS2_SHELL)"
    . "$ROS2_SETUP"
else
    echo "failed to set up ros2"
    return 1
fi
