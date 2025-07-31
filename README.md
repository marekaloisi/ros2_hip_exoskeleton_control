
# 🦿 ROS 2 Exoskeleton Control – Startup Tutorial

This repository contains the ROS 2 system for controlling a hip exoskeleton using two Maxon EPOS4 motor drives (left and right hip).  
The system receives predicted gait parameters and sends position commands to the motors using the CANopen protocol.  
It uses **ROS 2 lifecycle nodes**, so correct startup order and timing are **critical**.

---

## ✅ Requirements

- ROS 2 Humble installed
- This workspace is built and sourced
- 2 terminals
- **Both terminals must be run as root** (`sudo -E -s`)
- 24–48 V power supply connected to the EPOS4 drives (initially **OFF**)
- CAN interfaces correctly set up (`can0`, `can1`)
- Drives correctly wired and terminated

---

## 🔧 Step-by-Step Startup Guide

### 1. Open 2 terminals and become root in both

Run the following in **both terminals**:

```bash
sudo -E -s
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
cd ~/epos4_workspaces/epos4_basic_communication
source install/setup.bash
```

### 2. Terminal 1 – Launch CAN + lifecycle nodes

```bash
ros2 launch epos4_bringup can.launch.py
```

This starts the lifecycle nodes:
- `/master`
- `/joint_hip_left`
- `/joint_hip_right`

They will be in the **unconfigured** state. No hardware will initialize yet.

### 3. Terminal 2 – Run lifecycle activation script

```bash
bash start_both.sh
```

This script:
- Configures and activates `/master`
- Configures and activates both joint motor nodes

You will now see:

```
[joint_hip_left] Waiting for device to boot...
[joint_hip_right] Waiting for device to boot...
```

### 4. Turn on motor power supply (24–48 V)

As soon as you see the logs above, **turn on the motor power supply**.  
Recommended voltage: **36–42 V**  
Do **not** turn it on before step 3 — or the drives won’t boot properly.

### 5. Terminal 2 – Launch position control

```bash
ros2 launch epos4_bringup position_control.launch.py
```

This starts:
- Left and right position controller nodes
- Optionally: Gait prediction and gait manager nodes, if enabled in the launch file

---

## 🧠 Launch Sequence Summary

| Step | Command                         | Terminal | When                          |
|------|----------------------------------|----------|--------------------------------|
| 1    | `can.launch.py`                 | 1        | First                         |
| 2    | `start_both.sh`                 | 2        | After CAN nodes are up        |
| 3    | Turn on motor power             | –        | When "waiting for device" log |
| 4    | `position_control.launch.py`    | 2        | After power on + activation   |

---

## 🛠️ Troubleshooting

- Run **everything as root** (`sudo -E -s`)
- Power must be turned on **after** lifecycle activation
- Check node states with:
  ```bash
  ros2 lifecycle get /joint_hip_left
  ros2 lifecycle get /joint_hip_right
  ```
- If motors don't respond:
  - Verify power is reaching the drives
  - Ensure CAN bus is properly terminated (120 Ω)
  - Restart the system from scratch

---

## 📦 Repository Structure (simplified)

- `epos4_node_position_controller_left/` – Controls left hip motor
- `epos4_node_position_controller_right/` – Controls right hip motor
- `epos4_gait_parameter_prediction_node/` – Loads LSTM models and publishes gait parameters
- `epos4_gait_manager/` – Coordinates gait flag switching between legs
- `epos4_bringup/` – Launch files and startup scripts
- `epos4_interfaces/` – Custom ROS 2 message definitions

---
