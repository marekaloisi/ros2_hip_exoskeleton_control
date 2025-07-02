#!/bin/bash

# File: start_epos4.sh
# Usage: ./start_epos4.sh

set -e

echo "Configuring /master..."
ros2 lifecycle set /master configure

echo "Activating /master..."
ros2 lifecycle set /master activate

echo "Configuring /joint_hip_right..."
ros2 lifecycle set /joint_hip_right configure

echo "Activating /joint_hip_right..."
ros2 lifecycle set /joint_hip_right activate

echo "All done."
