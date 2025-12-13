# GO1 Simulation Setup Guide (ROS 2 Jazzy + Gazebo)

This document describes the full procedure to run the GO1 simulation using
`go1_simulation`, activate all required `ros2_control` controllers, and control
the robot posture using `junior_ctrl`.

---

## 1. Initial Setup (Run Once)

### 1.1 Update system packages
```bash
sudo apt update
sudo apt upgrade
```

### 1.2 Update rosdep
```bash
rosdep update
sudo apt update
```

### 1.3 Source ROS 2 Jazzy
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. Workspace Build (Run After Any File Change)

From the workspace root:

```bash
colcon build
source install/setup.bash
```

---

## 3. Launch Simulation (Terminal 1)

Open a new terminal and run:

```bash
source install/setup.bash
ros2 launch go1_simulation go1.gazebo.launch.py \
  world_file_name:=hospital.world \
  use_gt_pose:=true \
  spawn_delay:=40.0
```

Notes:
- `spawn_delay:=40.0` ensures the robot is spawned **after** the Gazebo world
  is fully loaded.
- Wait approximately **40–50 seconds** until the robot appears in **both
  Gazebo and RViz**.

---

## 4. Activate Controllers (Terminal 2)

Open a new terminal and run:

```bash
source install/setup.bash

for c in \
  FL_thigh_controller FR_thigh_controller RR_thigh_controller RL_thigh_controller \
  FL_calf_controller  FR_calf_controller  RR_calf_controller  RL_calf_controller \
  FL_hip_controller   FR_hip_controller   RR_hip_controller   RL_hip_controller \
  joint_state_broadcaster
do
  ros2 control load_controller --set-state active $c
done
```

Wait until all controllers are successfully activated.

This step is required to:
- Publish `/joint_states`
- Enable joint-level control
- Allow posture and motion commands to take effect

---

## 5. Run Posture Controller (Terminal 3)

Open a new terminal and run:

```bash
ros2 run unitree_guide2 junior_ctrl
```

Inside the `junior_ctrl` console, press the following keys **slowly and in order**:

```
1 → 2 → 5
```

This sequence transitions the robot into a stable standing posture.

---

## Full Execution Summary

1. (Once) System + rosdep update, source ROS 2 Jazzy  
2. (After changes) `colcon build` and source workspace  
3. Terminal 1: Launch Gazebo + RViz with spawn delay  
4. Terminal 2: Activate all `ros2_control` controllers  
5. Terminal 3: Run `junior_ctrl` and input `1 → 2 → 5`

After completing all steps, the GO1 robot should be fully operational in
Gazebo and RViz.
