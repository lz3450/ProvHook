#!/usr/bin/env bash
#
# clean.sh
#

set -e
set -o pipefail
# set -u
# set -x

umask 0022

################################################################################

rm -rf build install log
