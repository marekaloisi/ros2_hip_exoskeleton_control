#!/bin/bash

# File: start_epos4.sh
# Usage: ./start_epos4.sh

set -e


echo "Building Control Node..."

colcon build --packages-select epos4_gait_manager --cmake-clean-cache --event-handlers console_direct+ --parallel-workers 1 --allow-overriding epos4_gait_manager
colcon build --packages-select epos4_node_position_controller_right --cmake-clean-cache --event-handlers console_direct+ --parallel-workers 1 --allow-overriding epos4_node_position_controller_right
colcon build --packages-select epos4_node_position_controller_left --cmake-clean-cache --event-handlers console_direct+ --parallel-workers 1 --allow-overriding epos4_node_position_controller_left


source install/setup.bash
