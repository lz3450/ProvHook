#!/usr/bin/env bash
#
# distclean.sh
#

set -e
set -o pipefail
# set -u
# set -x

umask 0022

################################################################################

rm -rf build install log src
sudo rm -rf build-opt log-opt
