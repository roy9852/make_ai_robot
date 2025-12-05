# CRAIP: Creating Robot Artificial Intelligence Project

CRAIP is the final project for Seoul National University's **"How to Make a Robot with Artificial Intelligence (M2177.002600), Fall 2025"** course.

CRAIP is developed using the Language-Image-based Mobile Robot (LIMBO) framework, designed to enable robots to perform navigation tasks based on language instructions.

It contains a collection of ROS2 packages for controlling the Unitree Go1 quadruped robot in Gazebo simulation environment.

---

# ⚠️ Caution (Only After Git Clone)
This repository is currently under active development and **frequent updates** may occur.

To ensure you are working with the **latest version** of this project, please run the following command in your terminal:

```bash
cd make_ai_robot
git fetch origin
git status

# Update your workspace (Be careful to use)
git pull origin main
```

### ⚠️ Important Note:

Be careful not to overwrite your local changes when pulling updates. We recommend that you create new packages and work on them.

---

# 🔧 Prerequisites

- **Operating System**: Ubuntu 24.04
- **ROS2 Distribution**: Jazzy Jalisco
- **Gazebo**: Harmonic

---

# 📦 Packages Overview

## 1. **go1_simulation**
Gazebo Harmonic simulation environment for the Unitree Go1 robot with full ROS2 integration.
- URDF/Xacro robot description
- ROS2 control integration
- Ground truth pose publisher
- Keyboard teleoperation
- Map publisher and generator

## 2. **path_tracker**
MPPI (Model Predictive Path Integral) based path tracking controller for mobile robots.
- C++ implementation of MPPI algorithm
- Python example script for path generation
- Real-time trajectory tracking

## 3. **ros2_unitree_legged_controller**
ROS2 control interface for Unitree legged robots.
- Custom ROS2 controller plugin
- Joint-level control interface
- Compatible with ros2_control framework
- Hardware abstraction layer

## 4. **ros2_unitree_legged_msgs**
Custom ROS2 message definitions for Unitree robots.
- Motor command/state messages
- High-level command/state messages
- IMU, BMS, and sensor messages
- Compatible with Unitree SDK

## 5. **unitree_guide2**
High-level locomotion controller for Unitree Go1 robot.
- Finite State Machine (FSM) for gait control
- Balance control and estimation
- Trotting gait implementation
- ROS2 integration for simulation

## 6. **environment**
Worlds and models for simulation.
- World files like hospital.world, empty.world, etc.
- Models which will be used in the world files

## 7. **custom_interfaces**
Custom interfaces (topic or service) for your mission
- Currently, there is one service `LanguageCommand.srv` which will be used by `language_command_handler` package
- This service is used to command robot and get the ROS2 node (launch) name the robot called.

## 8. **language_command_handler**
Interface package to enable language based communication with robot.
- This packge is used to control robot with only language.
- Instead of starting new nodes or launch files for every case, you need to handle all the cases with only this package.

---

# 📥 Installation

**If you want to use Docker (optional)**: Please refer to [docker/README.md](docker/README.md).

## Step 1: Clone the Repository

```bash
cd ~
git clone https://github.com/roy9852/make_ai_robot.git
```

## Step 2: Install Dependencies

Use `rosdep` to automatically install all required dependencies:

```bash
# Update apt list and upgrade
sudo apt update
sudo apt upgrade

sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update

# Update apt list again (ROS2 packages)
sudo apt update

cd ~/make_ai_robot
# This command will take a while 
# Run 'source /opt/ros/jazzy/setup.bash' if there is no $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

This will install:
- Gazebo Harmonic and ROS2 integration (`ros_gz_sim`, `ros_gz_bridge`)
- All ROS2 packages (rclcpp, rclpy, tf2, ros2_control, etc.)
- System dependencies (Eigen, Boost, etc.)

### Step 3: Build the Workspace

```bash
colcon build
```

You should see:
```bash
Starting >>> ros2_unitree_legged_msgs
Starting >>> aws_robomaker_hospital_world
Starting >>> custom_interfaces
Starting >>> path_tracker                                        
Finished <<< aws_robomaker_hospital_world [1.64s]                             
Finished <<< custom_interfaces [7.23s]                                            
Starting >>> language_command_handler
Finished <<< language_command_handler [1.93s]                                     
Finished <<< ros2_unitree_legged_msgs [10.5s]                                        
Starting >>> ros2_unitree_legged_control
Starting >>> unitree_guide2                                    
Finished <<< ros2_unitree_legged_control [12.7s]                         
Starting >>> go1_simulation
Finished <<< path_tracker [23.9s]                                        
Finished <<< go1_simulation [2.34s]                                       
Finished <<< unitree_guide2 [20.0s]                        

Summary: 8 packages finished [30.6s]

```

## Step 4: Source the Workspace

```bash
source install/setup.bash
```

Add to your `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/make_ai_robot/install/setup.bash" >> ~/.bashrc
```

**Note:**

By default, ROS2 should be sourced first: `source /opt/ros/jazzy/setup.bash`. 

Keep in mind that if you have edited your source code, you need to rebuild and source `setup.bash` of your packages.


## Step 5: Understand the Workspace Structure

After building, your workspace will have these directories:

```
make_ai_robot/
├── src/                    # Source code (you edit files here)
│   ├── go1_simulation/
│   ├── path_tracker/
│   ├── environment/
│   ├── custom_interfaces/
│   ├── language_command_handler/
│   ├── ros2_unitree_legged_controller/
│   ├── ros2_unitree_legged_msgs/
│   └── unitree_guide2/
├── build/                  # Temporary build files (auto-generated)
├── install/                # Compiled packages (auto-generated)
├── log/                    # Build logs (auto-generated)
└── docker/                 # For docker users
```

**Note:**
- Only edit files in `src/` directory
- Never manually edit files in `build/`, `install/`, or `log/`
- If build fails, you can safely delete `build/`, `install/`, and `log/` and rebuild the packages
- Always source `install/setup.bash` after building

---

# 🚀 Usage

## 1. Launch Go1 Simulation in Gazebo

Start the complete simulation environment:

```bash
ros2 launch go1_simulation go1.gazebo.launch.py use_gt_pose:=true
```

You should see both Gazebo and RViz:

In RViz, you can see a colored 3D pointcloud from the RGB-D camera and a red 2D pointcloud from the LiDAR


<img src="images/start_gazebo.png?v=1" alt="Gazebo Simulation" width="600"/>


<img src="images/start_rviz_1.png?v=1" alt="RViz Visualization 1" width="600"/>


**Launch Arguments:**
- `use_gt_pose` - Use Ground Truth (GT) pose of "trunk" link for localization (Data is from Gazebo)
- `world_file_name:=<world_name>.world` - Choose world (hospital, empty, cafe, house) (Default: hospital)
- `x:=0.0 y:=1.0 z:=0.5` - Initial robot position (meter) (Default: x=0.0, y=1.0, z=0.5)
- `roll:=0.0 pitch:=0.0, yaw:=0.0` - Initial robot orientation (radian) (Default: roll=0.0, pitch=0.0, yaw=0.0)

**Example:**
```bash
ros2 launch go1_simulation go1.gazebo.launch.py use_gt_pose:=true world_file_name:=empty.world x:=-1.0 y:=0.0 yaw:=1.57
```

### Real Time Factor

If your computer does not have a GPU, the simulation may be too slow, which will prevent the robot from walking properly. You should check the Real Time Factor (RTF) in the Gazebo simulation. It should be at least 50%. 

<img src="images/Gazebo_RTF.png" alt="Gazebo RTF" width="600"/>

Launch file, by default, tries to use GPU for Gazebo simulation. To check whether GPU is being used, run this command:
```bash
nvidia-smi
```

If your computer performance is limited, try to use `empty.world` first.

### ⚠️ Important Note:

Currently, you are using the ground truth pose of the robot from the simulator. However, **you should implement your own localization module without relying on ground truth data.** For the final project, it is not allowed to use any ground truth values. All information must come from sensors (RGB-D Camera, LiDAR, IMU, and map), and you need to estimate the robot's state from these sensors.

## 2. Visualize pre-built 2D occupancy grid map

To visualize the map in RViz, run this command:
```bash
ros2 launch go1_simulation visualize_map.launch.py
```

In RViz, you should see:

<img src="images/start_rviz_2.png?v=1" alt="RViz Visualization 2" width="600"/>

You can compare the pointcloud (RGB-D and LiDAR) with the pre-built map

## 3. Unitree Guide Controller

Run the high-level locomotion controller:

```bash
ros2 run unitree_guide2 junior_ctrl
```

After you run this node, press a number key (1-5) to change the robot's state in the same terminal

**Robot States (FSM States)**
- **Number 1**: Passive mode - Robot is passive, no joint control (initialization)
- **Number 2**: Fixed stand - Robot stands up with fixed position
- **Number 3**: Free stand - Robot stands up with balance control (can adapt to external forces)
- **Number 4**: Trotting - Robot performs trotting gait (not recommended without proper setup)
- **Number 5**: Move base - Robot can move using `/cmd_vel` commands (velocity control mode)

**Operation Order After Running Node (5 Steps):**
- **Step 1**: Spawn the robot in Gazebo environment and start `junior_ctrl` node
- **Step 2**: Press number 1 for initialization
- **Step 3**: Press number 2 to stand up the robot
- **Step 4**: After the robot stands up, press number 5. Then the robot's body will drop slightly
- **Step 5**: Now, you can move your robot with `/cmd_vel` topic

Please watch below video


<img src="images/use_junior_ctrl.gif" alt="Robot Control Demo" width="600"/>


### Keyboard Teleoperation

Control the robot with keyboard after running `junior_ctrl` node:

```bash
ros2 run go1_simulation move_go1_with_keyboard.py
```

You can also use a CLI command (this command is for rotation):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" -r 10
```

## 4. Path Tracking

**Step 1**: Launch the MPPI path tracker:

```bash
ros2 launch path_tracker path_tracker_launch.py
```

**Step 2**: Generate and follow a path to specific pose:

```bash
# Move to specified 2D pose 
# Run this code, wait a few seconds
ros2 run path_tracker move_go1.py 
# Enter desired 2D position (x, y, yaw): ex) 5 1 0
```

**Step 3**: Compare goal pose and real pose!:
```bash 
# Read x, y, z, qx, qy, qz, qw and compare with your command
# You need to convert the quaternion to a yaw angle
ros2 topic echo /go1_pose

# Or you can use
ros2 topic echo /go1_pose_2d
```

**How it works:**
1. The path tracker subscribes to `/local_path` (desired path)
2. It uses `/go1_pose` (current robot position) for feedback
3. It publishes `/cmd_vel` commands to move the robot
4. MPPI algorithm optimizes the control commands in real-time

**Note:**
- Make sure the robot is in "move base" mode (press 5 in `junior_ctrl` terminal)
- The robot will generate a smooth curved path to the target

### ⚠️ Important Note:

The current `move_go1.py` does not consider collisions at all. **You should implement your own path planner for collision avoidance**. The planned path will be passed to `path_tracker_launch.py`, and the robot will follow that path.

Also, the path tracking ability might not be optimal. **For better path following, you can tune the parameters in `path_tracker/config/mppi.yaml`.**

## 5. Communicate with robot through language

**Step 1**: Download Anaconda installation script

To handle language command, we will use OpenAI API. 
For that, we need to create new virtual environment with Anaconda. 

```bash
cd ~/Downloads
# This command will create 'Anaconda3-2024.10-1-Linux-x86_64.sh' under Downloads folder
wget https://repo.anaconda.com/archive/Anaconda3-2025.06-1-Linux-x86_64.sh
```

**Step 2**: Install Anaconda

```bash
bash Anaconda3-2025.06-1-Linux-x86_64.sh
```

You should see:

```bash
Welcome to Anaconda3 2025.06-1

In order to continue the installation process, please review the license
agreement.
Please, press ENTER to continue
>>>
```

Enter `yes` and press `enter` several times to finish installation.

You can check the installation with:

```bash
source ~/.bashrc
conda --version
# You should see: conda 25.5.1
```

**Step 3**: Edit `~/.bashrc` file

Turn off conda if it is enables:
```bash
# If your terminal is like: (base) roy@roy:~$ 
conda deactivate
```

Open `~/.bashrc`. We use `gedit` here:
```bash
gedit ~/.bashrc
```

Change Anaconda setting:
```bash
# Remove this:

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/roy/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0]; then
...
...
...
...
unset __conda_setup
# <<< conda initialize <<<

# And add this:
source ~/anaconda3/etc/profile.d/conda.sh
```

You need to also declare `OPENAI_API_KEY` to use ChatGPT through Python API:
```bash
# Add this in your bashrc file:
export OPENAI_API_KEY='sk-........' # Use your API key
```

**Step 4**: Create new conda environment

Create new conda environment `language_command_handler` to use `language_command_handler` package.

Run this command:
```bash
source ~/.bashrc

# Press yes
conda create --name language_command_handler python=3.12.3
```

Install dependencies:
```bash
cd ~/make_ai_robot/src/langauge_command_handler
conda activate language_command_handler
pip install -r requirements.txt
```

If there is no error, you can use `language_command_handler` package from now.

**Step 5**: Control robot with language

Start simulation and junior controller:
```bash
# Do not activate conda before ros2 command, it can occur error 
# Terminal 1
ros2 launch go1_simulation go1.gazebo.launch.py use_gt_pose:=true

# Terminal 2
ros2 run unitree_guide2 junior_ctrl
```

After make robot setting as 'Move base' with number 5, run this command to start `language_command_handler` node:
```bash
source ~/.bashrc
ros2 launch language_command_handler start_command_handler.launch.py
```

Now you can move various language command to control the robot. 
For example, to move robot in forward direciton, you can use ROS2 service:
```bash
source ./install/setup.bash
ros2 service call /language_command custom_interfaces/srv/LanguageCommand "{command: 'go forward'}"
```

You can use various commands like:
```bash
ros2 service call /language_command custom_interfaces/srv/LanguageCommand "{command: 'move behind'}"

ros2 service call /language_command custom_interfaces/srv/LanguageCommand "{command: 'freeze'}"

ros2 service call /language_command custom_interfaces/srv/LanguageCommand "{command: 'There is a bear behind you. You should run away'}"
```

The process of `language_command_handler` is like this:

1. LLM prompt and callable ROS2 nodes and launch files are listed in `language_command_handler/config/command_handler_config.yaml` 
2. Take language command from the service
3. Call LLM with prompt, callable action list, and your command. LLM will select the best action to accomplish your command
4. It start ROS2 node (or launch file) choosen before. If there was previous action, it automatically stops previous one and start the new one.

Currently there are 3 actions you can call:
- go_front.py
- stop.py
- go_back.launch.py

You should change `command_handler_config.yaml` as you want to run your own codes with only language command.

---

# 📝 Final Project:

## 💯 Scoring Structure

The final project consists of two parts: **Module Design** and **Competition**.

**Total: 100 points**
- **Module Design**: 30 points
- **Competition**: 70 points

**Note**: Projects are evaluated on a team basis, not individually.

### Module Design (30 points)

You will build three essential modules for navigation:

| Module | Points | Package Name |
|--------|---------|--------------|
| Localization | 20 pts | `localization` |
| Perception | 10 pts | `perception` |

Even though path planning is essential for navigation, because we provided enough source code through eTL, we do not use it for grading.

### Competition (70 points)

**You will integrate all modules into a complete navigation system to accomplish 6 missions.**
You will be given 30 minutes to accomplish the missions. 
If the scores of two teams are same, the team which used lower time is score higher.

Sometimes robot dies abrupty. In that case, we stop the timewatch.
We manually move the robot to the place the robot died and restart the simulation. And time is measured again.
To deal with this kind of sudden restart, it is highly recommend to make your modules to take robot start arguments.
Especially, for the localization module, if robot starts not near `x=0, y=1, yaw=0`, and you do not used robot initialization with ROS2 arguments, even though you have proper map data, the localization would not work.

**All the command should be done by only language command. So you can not manually turn on and off ROS2 node/launch.** To see how to do that, please refer how `language_command_handler` works. 

- **6 missions** × 10 points each = 60 points
- **Bonus**: +10 points for the most challenging mission (determined by lowest success rate)
- **Total**: 70 points

---

## Module Design 1. `localization` 

**Goal**: Create a `localization` package for particle filter-based robot localization using available sensors and map data.

**Why this matters**: 

The path tracker relies on accurate robot pose estimation. Without robust localization, the robot cannot determine its position and will be unable to follow paths correctly.

**What you need to do:**

Create a ROS2 package named `localization` with nodes (Python or C++) that:

1. **Subscribe to sensor and map data**:
   - LiDAR data (`/scan`)
   - IMU data (`/imu_plugin/out`)
   - Map data (`/map`)
   - Optionally: Camera data for visual odometry

2. **Estimate the robot's 3D pose**:
   - Implement a particle filter for localization
   - Fuse sensor data to improve accuracy
   - Handle uncertainty and noise in sensor measurements

3. **Publish the estimated pose**:
   - Topic: `/go1_pose` 
   - Message type: `geometry_msgs/msg/PoseStamped`
   - Frame ID: `map`

4. **Broadcast TF transformations**:
   - `map` → `odom`: Corrected odometry based on localization
   - `odom` → `base`: Odometry from robot motion
   
   This maintains the standard ROS2 navigation transform tree.

**Related Topics:**
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/go1_pose` | geometry_msgs/msg/PoseStamped | Robot pose (x, y, z, qx, qy, qz, qw)|
| `/tf` | tf2_msgs/msg/TFMessage | Transformation tree information between links |
| `/map` | nav_msgs/msg/OccupancyGrid | 2D occupancy grid map for localization |
| `/scan` | sensor_msgs/msg/LaserScan | 2D Lidar data |
| `/imu_plugin/out` | sensor_msgs/msg/Imu | IMU measurements |
| `/camera_face/camera_info` | sensor_msgs/msg/CameraInfo | Camera information of face camera |
| `/camera_face/image` | sensor_msgs/msg/Image | RGB image of face camera |
| `/camera_face/depth` | sensor_msgs/msg/Image | Depth image of face camera |
| `/camera_face/points` | sensor_msgs/msg/PointCloud2 | Colored point cloud of face camera |
| `/camera_top/camera_info` | sensor_msgs/msg/CameraInfo | Camera information of top camera |
| `/camera_top/image` | sensor_msgs/msg/Image | RGB image of top camera |
| `/camera_top/depth` | sensor_msgs/msg/Image | Depth image of top camera |
| `/camera_top/points` | sensor_msgs/msg/PointCloud2 | Colored point cloud of top camera |

**Learning Steps:**

1. **Explore available sensors**:
   - Use the following commands to inspect sensor topics:
     - `ros2 topic list` - List all available topics
     - `ros2 topic info <topic_name> --verbose` - Get detailed topic information
     - `ros2 topic echo <topic_name>` - View real-time topic data
     - `ros2 topic hz <topic_name>` - Check publishing frequency
  
2. **Study reference code**:
   - **`go1_gt_pose_publisher.py`**: Understand how to:
     - Publish `PoseStamped` messages
     - Broadcast TF transforms using `tf2_ros`
     - Structure the transform tree (`map`, `odom`, `base` frames)
   - **`publish_pointcloud.py`**: Learn how to:
     - Subscribe to ROS2 topics
     - Process sensor data
     - Publish processed data

3. **Study relevant algorithms**: 
   - **Particle Filter**: Monte Carlo Localization (MCL)
   - **Scan Matching**: Iterative Closest Point (ICP)
   - **Sensor Fusion**: IMU pre-integration, Extended Kalman Filter
   - **Map Representation**: Occupancy grid interpretation

4. **Implement your localization package**:
   - Start with a basic particle filter
   - Integrate LiDAR scan matching
   - Add IMU for motion prediction
   - Tune parameters for accuracy and performance

### ⚠️ Important Requirements: 

1. **No Ground Truth Data**: 
   - Do **not** use ground truth poses from Gazebo (e.g., `go1_gt_pose_publisher.py`)
   - Estimate the pose using only sensor data attached to the robot
   
2. **No External Navigation Packages**: 
   - Do **not** use existing navigation packages like Nav2's AMCL
   - You must implement your own localization algorithm from scratch
   - Standard ROS2 libraries (tf2, sensor message parsing, etc.) are allowed

3. **Support Variable Initial Poses**: 
   - Add launch arguments for initial pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`)
   - Default spawn: `x=0, y=1, z=0.5` with `roll=0, pitch=0, yaw=0`
   - Your localization must initialize correctly from any starting pose specified via arguments
   - **This is crucial for the case when you restart the simulation from another starting pose**

### 📊 Evaluation:

**Points**: 20 points (success/failure)

**Evaluation Process**: 

The TAs will provide 4 robot trajectories using `ros bag`. Each ros bag includes camera, lidar, and IMU topics. For the localization, you need map topic also, but we do not provide it. You might use the coarse map we provide, or your custom one. 

Your localization module should localize the robot using these data. **For the ros bag data, robot always start near the `x=0, y=1, yaw=0`.** For each trajectory (for each ros bag), you should submit your localization result as `.txt` file. We provide one file as an example. **So you should submit 3 `.txt` files.** For the submission format, please refer the txt file we provide.

**Evaluation Metrics**:

For the evaluation, TAs will use Absolute Trajectory Error (ATE) for the metric.
Your localization result txt will be compared to ground truth trajectory.
If your accuracy is above the threshold we set, your localization modules is graded as success.
There are 3 ros bag data, whose score of each is (4, 8, 8), total 20. If you success, you get the point. 

---

## Module Design 2. `perception` 

This module focuses on building a **ROS2 perception system** capable of detecting objects, classifying them, estimating distance, and generating robot speech cues based on object position. The final output will be verified using the provided interface viewer.

**Goal**: Develop a ROS2 package named **`perception`** that performs real-time object detection using RGB + depth images and publishes formatted outputs for the system.

**Requirements**

1. Subscribe to Camera Data

   Your node must subscribe to:

   - **RGB Image Topic**: `/camera_top/image`
   - **Depth Image Topic**: `/camera_top/depth`
   - **Point Cloud Topic**: `/camera_top/points`
   - **Camera Info**: `/camera_top/camera_info`

2. Object Detection & Classification

   Your perception node should:

   - Run an object detection model such as YOLO
   - Determine whether the target object exists
   - Use the depth image to obtain the distance to the target object

3. Required Published Topics

   The node must publish the following:

   | Topic | Type | Description |
   |------|------|-------------|
   | `/camera/detections/image` | `sensor_msgs/Image` | Original RGB image with bounding boxes |
   | `/detections/labels` | `std_msgs/String` | Detected object label(s) |
   | `/detections/distance` | `std_msgs/Float32` | Distance to the detected object |
   | `/robot_dog/speech` | `std_msgs/String` | `"bark"` if an edible object is centered, otherwise `"None"` |

   #### Center Region Rule

      - The image's leftmost and rightmost **1/5** regions are *excluded*.
      - If an edible object is detected whose bounding-box center lies within the **middle 3/5** of the image → publish `"bark"`.

4. Interface Verification

   <img src="images/perception_module_example.png" alt="Perception Module Example" width="600"/>

   Run the viewer to verify all published topics:

   ```bash
   ros2 launch module_test interface_viewer.launch.py
   ```


**Learning Steps:**
1. **Study computer vision and machine learning**: 
   - **Object Detection**: YOLO, DETR, etc
   - **Data Collection**: Capture images from simulation, label using tools like LabelImg or Roboflow
   - **Model Training**: Fine-tune pre-trained models on custom food dataset
   - **Depth Processing**: Extract distance from depth images, handle invalid depth values
   - **Visual Servoing**: Align robot with object using visual feedback

2. **Implement your perception package**:
   - Start with a pre-trained object detection model
   - Collect training data for good/bad food from the simulation
   - Fine-tune the model for your specific objects
   - Integrate depth sensing for distance estimation
   - Implement control logic for approaching and centering

---
## 📊 Evaluation: 10 points (absolute evaluation, pass/fail)

### Overview
Two ROS bag files will be provided. For each bag file, you will replay the data, run the provided launch file, and **record the viewer output as a video**. Any screen-recording software (e.g., SimpleScreenRecorder, OBS, etc.) may be used.

### Bag File Description
Each ROS bag includes a sequence in which the robot **rotates in place**, capturing its surroundings. The images contain **multiple objects that appear in the actual competition environment**. Your system must process these scenes to detect objects and determine when the barking condition is met.

**Download Link**: [Perception Module Test Bag Files](https://drive.google.com/file/d/1aQZ1qr5Q9m-JcWN4qgewoQ4dtE0N_2QQ/view?usp=sharing)

### Task Requirement
When an object is located **within 3 meters** of the robot and appears clearly in view, your system must fulfill the following:

#### 1. Object Detection
   - The system must accurately detect objects within a 3-meter radius.  
   - All **intermediate outputs** (detection results, distance estimation) must be clearly and correctly visualized in the viewer.

#### 2. Bark Topic Publishing
   - When detected objects within 3 meters are centered in the view,  
   - The system must publish the **`/bark`** topic at the **appropriate moment**.

### Submission
   - Submit a **screen-recorded video** showing the full viewer output.  
   - Submit **one video for each bag file** (two videos total).  

### Evaluation Criteria
Evaluation follows an **absolute and qualitative** standard:

   - Detection of objects within the scene
   - Reasonable distance estimation (does not need to be perfectly accurate)
   - Correct timing of `/bark` topic publication when an object is centered within 3 meters  
   - Evaluation is pass/fail
---

# ⚔️ Competition


In the competition, you will integrate all three modules (localization, path planning, perception) into a complete autonomous navigation system. The robot will receive high-level language commands and must accomplish various missions.

**You will integrate all modules into a complete navigation system to accomplish 6 missions.**
You will be given 30 minutes to accomplish the missions. 
If the scores of two teams are same, the team which used lower time is score higher.

Sometimes robot dies abrupty. In that case, we stop the timewatch.
We manually move the robot to the place the robot died and restart the simulation. And time is measured again.
To deal with this kind of sudden restart, it is highly recommend to make your modules to take robot start arguments.
Especially, for the localization module, if robot starts not near `x=0, y=1, yaw=0`, and you do not used robot initialization with ROS2 arguments, even though you have proper map data, the localization would not work.

**All the command should be done by only language command. So you can not manually turn on and off ROS2 node/launch.** To see how to do that, please refer how `language_command_handler` works. 

**The language would be changed!** For example, for mission 1, we will use similar, but not exactly same language command. For example, instead of 'Please navigate me to the toilet', we will might use 'Toilet, toilet, toilet. Hurry.' for the language command. The robot should do the mission robustly to the langauge command. **For that, you need to design your prompt well with `command_handler_config`.**

**Key Rules**:
- You **must** use your own localization module (no ground truth robot poses)
- You **must** use your own path planning module for collision-free navigation  
- Using ground truth object poses from Gazebo is **not allowed**
- **Missions will be done only via natural language commands (e.g., "Please navigate me to the toilet")**

---

## Mission 1: Navigate to the Toilet

**Goal**: Autonomously navigate the robot to the toilet location in the hospital environment.

**Task Description**: 

The robot will receive a natural language command (e.g., "Go to the toilet"). Your system must navigate the robot to the toilet. The toilet should be aligned to camera center. The navigation stop when robot `bark` to alert the position of toilet.

**Approach**: 

For this mission, you may **pre-calculate and hard-code** the toilet's position:
- Explore the map before the competition
- Determine the 2D pose that faces toilet directly
- Example: `x=10.0, y=5.0, yaw=1.57` (this is just an example, not the actual position)
- Use your path planning to navigate to this pre-defined goal

**Requirements**:
- Robot must `bark` when the conditions (distance and orientation) are satisfied. Condition for barking is same as `module design 3`.
- Navigation must be collision-free for high rate success. But we do not degrade the score with this. 
- You must use your own localization and path planning modules

### 📊 Evaluation:

**Points**: 10 points with success/failure. The robot should bark when conditions are satisfied.


<img src="images/mission_1.png" alt="Mission 1" width="600"/>


## Mission 2: Find and Identify Edible Food

**Goal**: Search the environment, find edible food, and bark in front of it.

**Task Description**: 

The robot will receive a natural language command (e.g., "Find something to eat"). Your system must:
1. Explore the hospital environment
2. Detect good food using the perception module
3. Bark in front of it

**Key Difference from Mission 1**: 

⚠️ **Food positions are NOT fixed** - We will change the position of foods for competition. You **cannot** pre-calculate and hard-code positions like in Mission 1. The robot must actively search and detect food using its cameras. At each room, there would be at least one food. (good or bad)

**Food Categories**:

The following food items will be placed in the environment:

- **Left column (Good Food - Edible)**: Banana, Apple, Pizza
- **Right column (Bad Food - Not Edible)**: Rotten/spoiled versions, inedible objects

<img src="images/mission_2_1.png" alt="Mission 2 1" width="600"/>


<img src="images/mission_2_2.png" alt="Mission 2 2" width="600"/>


**Food Placement**: 

We provide example map with foods for practice. 
There are total 10 foods in the map. The hospital map has 20 rooms, and half of them (10 rooms) have the food each. 
We provide a map with food for your practice. 
But for the final competition, the position of the foods will be changed. (room change from room1 to room2, or position change within the room.)
The physical position (floor, table, etc.) does not matter. A good apple is edible whether on the floor or floating in the air - only the classification matters.


<img src="images/mission_2_3.png" alt="Mission 2 3" width="600"/>


<img src="images/mission_2_4.png" alt="Mission 2 4" width="600"/>



### 📊 Evaluation:

**Points**: 10 points with success/failure. The robot should bark when conditions are satisfied.

## Mission 3

**Goal**: Move to cone of specified color

**Task Description**:

The robot will receive a natural language command (e.g. "Go to the red cone"). Your system must navigate the robot to the specified colored cone. The navigation stops when robot `bark` to alert the position of the cone. It is not determined which color of cone will be used for the command. The robot should be able to detect any color of cone.


<img src="images/mission_3.png" alt="Mission 3" width="600"/>


**Requirements**:

There are three cones with different color (red, green, and blue). 
The positions of cones will not be changed. But the order might be different. 
For example, currently the order is blue cone, red cone, and green cone from left.
But that order might be changed to red, green, and blue.

### 📊 Evaluation:

**Points**: 10 points with success/failure. The robot should bark when conditions are satisfied.

---

## Mission 4

**Goal**: Move to box to the goal position

**Task Description**:

The robot will receive a natural language command (e.g. "Move the box to the goal position"). 
Your system must push the box to the goal position with the robot body. The goal position is specified as red area.


<img src="images/mission_4_1.png" alt="Mission 4 1" width="600"/>


**Requirements**:

The goal position (red area) will not be changed for the final competition.
But the position of delivery box will be changed. 
TAs will randomly choose position of delivery box among three candidates. 
Distance between goal area and delivery box is 2m, and the orientation of box will be aligned to make pushing easy.

<img src="images/mission_4_2.png" alt="Mission 4 2" width="600"/>


### 📊 Evaluation:

**Points**: 10 points with success/failure. The box should be in the red area.

---

## Mission 5

**Goal**: Move to the empty room without stop sign

**Task Description**:

The robot will receive a natural language command (e.g. "Move to the empty room"). 
There are two empty room in the hospital. You should go one of them without the stop sign.
The stop sign will be placed near the entrance of the room to notice whether this room is allowed to go or not.


<img src="images/mission_5_1.png" alt="Mission 5 1" width="600"/>


<img src="images/mission_5_2.png" alt="Mission 5 2" width="600"/>\


**Requirements**:
The position of the stop sign will be changed (in front of empty room1 or room2, how far from the entrance)

### 📊 Evaluation:

**Points**: 10 points with success/failure. The robot body should be in the room.


## Mission 6

**Goal**: Rotate around the nurse.

**Task Description**:

In the break room of the hospital, there is a nurse next to the couch. 
Find her, and rotate around her. She will be in the same room for the competition also, but the exact position might be different little bit. 


<img src="images/mission_6_1.png" alt="Mission 6 1" width="600"/>


<img src="images/mission_6_2.png" alt="Mission 6 2" width="600"/>\


**Requirements**:
The robot rotate around her. The rotation direction is not important.

### 📊 Evaluation:

**Points**: 10 points with success/failure. 

---

# 💡 Helpful Tips for ROS2 Beginners

## Useful ROS2 Commands

**Check available topics:**
```bash
ros2 topic list
```

**Monitor a topic in real-time:**
```bash
ros2 topic echo /go1_pose
```

**Check topic information:**
```bash
ros2 topic info /scan
```

**Visualize tf tree:**
```bash
ros2 run tf2_tools view_frames
# This creates a PDF file showing all transformations
```

**Record data for later analysis:**
```bash
ros2 bag record -a  # Record all topics
ros2 bag record /scan /go1_pose  # Record specific topics
```

**Play recorded data:**
```bash
ros2 bag play <bag_file>
```

**Check node information:**
```bash
ros2 node list
ros2 node info /path_tracker
```

## Working with Multiple Terminals

ROS2 typically requires multiple terminal windows. Here's a suggested workflow:

1. **Terminal 1**: Launch Gazebo simulation
2. **Terminal 2**: Run robot controller (`junior_ctrl`)
3. **Terminal 3**: Run additional nodes (keyboard control, path tracker, etc.)
4. **Terminal 4**: Monitor topics and debugging

💡 **Pro tip**: Use terminal multiplexers like `tmux` or `terminator` for easier management!

---

# 🔍 Troubleshooting

## Build Warnings
All packages are configured to build cleanly. If you see warnings, try to build again:
```bash
rm -rf build install log
colcon build
```

## Gazebo Not Found
Ensure Gazebo Harmonic is installed:
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-ros-gz
```

## Missing Dependencies
Re-run rosdep:
```bash
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

## "Command not found" Error
If ROS2 commands are not found, make sure you've sourced the ROS2 installation:
```bash
source /opt/ros/jazzy/setup.bash
source ~/make_ai_robot/install/setup.bash
```

## Robot Doesn't Walk Well
**Common causes:**
1. **Low Real Time Factor**: Check the RTF in Gazebo (displayed at the bottom right of the GUI). It should be above 50% for normal operation. If lower, the simulation is running too slow.
2. **Suboptimal MPPI parameters**: You may need to tune parameters in `path_tracker/config/mppi.yaml` for better locomotion.
3. **Low IMU frequency**: Check the frequency of `/imu_plugin/out` using `ros2 topic hz /imu_plugin/out`. It should be above 500 Hz for normal operation.
4. **Hardware limitations**: A more powerful computer (especially with a dedicated GPU) may be necessary.


<img src="images/Gazebo_RTF.png" alt="Gazebo RTF" width="600"/>


## Robot Doesn't Move
**Common causes:**
1. Robot is not in "move base" mode → Press `5` in the `junior_ctrl` terminal
2. Controller not running → Check if `junior_ctrl` node is running
3. Path tracker not receiving pose → Check `ros2 topic echo /go1_pose`
4. Simulation paused → Click play button in Gazebo

## Controller Not Loading
Check controller manager status after running `go1.gazebo.launch.py`:
```bash
ros2 control list_controllers
```

You should see 13 active controllers (12 joint controllers + 1 joint state broadcaster):
```bash
FL_thigh_controller     ros2_unitree_legged_control/UnitreeLeggedController  active
FR_thigh_controller     ros2_unitree_legged_control/UnitreeLeggedController  active
FR_hip_controller       ros2_unitree_legged_control/UnitreeLeggedController  active
RR_thigh_controller     ros2_unitree_legged_control/UnitreeLeggedController  active
RR_calf_controller      ros2_unitree_legged_control/UnitreeLeggedController  active
FL_calf_controller      ros2_unitree_legged_control/UnitreeLeggedController  active
RL_hip_controller       ros2_unitree_legged_control/UnitreeLeggedController  active
FR_calf_controller      ros2_unitree_legged_control/UnitreeLeggedController  active
RL_calf_controller      ros2_unitree_legged_control/UnitreeLeggedController  active
FL_hip_controller       ros2_unitree_legged_control/UnitreeLeggedController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster        active
RL_thigh_controller     ros2_unitree_legged_control/UnitreeLeggedController  active
RR_hip_controller       ros2_unitree_legged_control/UnitreeLeggedController  active
```

# 🗺️ Building a New Map

To create a new map instead of using the pre-built one, you can enable map generation mode.

**Step 1**: Start the simulation with the ground truth pose enabled:

```bash
ros2 launch go1_simulation go1.gazebo.launch.py use_gt_pose:=true
```

**Step 2**: In a separate terminal, launch the map visualization with map generation enabled:

```bash
ros2 launch go1_simulation visualize_map.launch.py generate_new_map:=true
```

After a few seconds, you should see RViz with an incrementally building map:


<img src="images/generate_map_1.png" alt="Map Generation 1" width="600"/>

**Step 3**: Move the robot around the environment to build the map:

Use keyboard teleoperation or send velocity commands to explore the environment. As the robot moves, the map will update incrementally.

<img src="images/generate_map_2.png" alt="Map Generation 2" width="600"/>

**Step 4**: Save the generated map:

Once you have explored the entire environment, save the map:

```bash
# First, stabilize the robot:
# In the junior_ctrl terminal, press '2' to enter 'fixed stand' mode
# This prevents issues during the map saving process

# Then, in a separate terminal, run:
ros2 run go1_simulation save_map.py
```

This will create two new files in the maps directory:

- `~/make_ai_robot/src/go1_simulation/maps/my_world.pgm` - The map image
- `~/make_ai_robot/src/go1_simulation/maps/my_world.yaml` - Map metadata (resolution, origin, etc.)

**Important**: 
The generated map should include all obstacles (furniture, people, etc.), not just walls. A complete map is essential for effective path planning and collision avoidance.