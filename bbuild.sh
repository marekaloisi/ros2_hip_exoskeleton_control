#!/bin/bash

# File: start_epos4.sh
# Usage: ./start_epos4.sh

set -e



echo "Building Bringup..."
colcon build --packages-select epos4_bringup --cmake-clean-cache --event-handlers console_direct+ --parallel-workers 1 --allow-overriding epos4_bringup


source install/setup.bash
