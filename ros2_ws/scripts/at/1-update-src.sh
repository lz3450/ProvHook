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

srcdir="$ROS2_WS/src/rt"

################################################################################
rm -rf "$srcdir"

################################################################################
get_src rt.repos "$srcdir"

###
rm -rf "$srcdir"/navigation2/nav2_system_tests

################################################################################
get_src deps.repos "$srcdir"

###
rm -rf "$srcdir"/create3_sim/{irobot_create_gazebo,irobot_create_ignition}
find "$srcdir"/joystick_drivers -mindepth 1 -maxdepth 1 -type d \
    ! -name 'joy' \
    ! -name 'joy_linux' \
    ! -name 'sdl2_vendor' \
    -exec rm -rf {} +

# rm -rf "$srcdir"/bond_core/{bond_core,bondpy,test_bond}
# rm -rf "$srcdir"/ros2_control/{doc,joint_limits_interface,rqt_controller_manager}
# rm -rf "$srcdir"/vision_opencv/{image_geometry,opencv_tests,vision_opencv}
# rm -rf "$srcdir"/diagnostics/{diagnostic_aggregator,diagnostic_common_diagnostics,diagnostic_remote_logging,diagnostics,self_test}
# find "$srcdir"/ros2_controllers -mindepth 1 -maxdepth 1 -type d \
#     ! -name 'diff_drive_controller' \
#     ! -name 'gripper_controllers' \
#     ! -name 'joint_state_broadcaster' \
#     ! -name 'joint_trajectory_controller' \
#     ! -name 'position_controllers' \
#     ! -name 'forward_command_controller' \
#     -exec rm -rf {} +
# find "$srcdir"/generate_parameter_library -mindepth 1 -maxdepth 1 -type d \
#     ! -name 'generate_parameter_library' \
#     ! -name 'generate_parameter_library_py' \
#     ! -name 'parameter_traits' \
#     -exec rm -rf {} +
# rm -rf "$srcdir"/create3_sim/irobot_create_common/{irobot_create_common_bringup,irobot_create_control,irobot_create_nodes,irobot_create_toolbox}
# find "$srcdir"/joystick_drivers -mindepth 1 -maxdepth 1 -type d \
#     ! -name 'joy' \
#     ! -name 'sdl2_vendor' \
#     -exec rm -rf {} +

###
echo "Successfully updated source code and applied patches"

################################################################################
get_dep_pkgs dep-pkgs.txt "$ROS2_WS/src"

###
echo "Successfully updated dependency package list"
