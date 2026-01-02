#!/usr/bin/env bash
#
# update-src.sh
#

set -e
set -o pipefail
# set -u
# set -x

umask 0022

################################################################################

ROS_DISTRO=humble
. ../lib.sh

srcdir="$ROS2_WS/src/ros2_humble"

################################################################################
rm -rf "$srcdir"

################################################################################
wget https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO/ros2.repos -O ros2_humble.repos
get_src ros2_humble.repos "$srcdir"

echo "Successfully fetched ROS2 $ROS_DISTRO source code"

################################################################################
get_dep_pkgs dep-pkgs.txt "$srcdir"

echo "Successfully updated dependency package list"
