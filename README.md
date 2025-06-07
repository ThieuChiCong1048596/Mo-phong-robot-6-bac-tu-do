
# 6-DOF Robot Simulation

## Overview

The project provides a simulation environment for a 6-degree-of-freedom (6-DOF) industrial robot using ROS Noetic, MoveIt, Gazebo, and MATLAB GUI interface. The system integrates a URDF model from SolidWorks, supporting:

* Motion planning (trajectory planning)
* Robot control (forward/inverse kinematics)
* Environment interaction (object detection and manipulation)

Main idea: Build a complete 6-DOF robot simulation system, connecting ROS ↔ MATLAB, processing images from sensors, and enabling flexible control via GUI or Python scripts.

## Features

* 6-DOF robot simulation: Convert SolidWorks Assembly files to URDF + Gazebo (ROS) with accurate joint simulation
* Motion planning: MoveIt, controlled via Python (IK_solver.py, Cartesian_path.py)
* MATLAB interface: Intuitive control via GUI (gui.mlapp)
* Object detection: Image processing with OpenCV (Detectobject.py)
* Environment interaction: Spawn objects in Gazebo (node_spawn_box_models_in_gazebo.py)
* End-effector control: Tracking and adjusting End-Effector (EE_tracker.py, node_set_predefined_pose.py)
* 3D design: Complete SolidWorks model

<div align="center">
  <img src="https://drive.google.com/uc?export=view&id=1YwCPNcOnOscLRBTUyOEucLvSFvD39UGx">
  <p align="center">Assembly Model in SolidWorks
</div>

<div align="center">
  <img src="https://drive.google.com/uc?export=view&id=1m1rPwmgqAXiJfKh5xN3QxK1OqHbtwVgE">
  <p align="center">3D Simulation Model in Gazebo
</div>

## Demo

Video simulation:

<div align="center">
  <a href="https://drive.google.com/file/d/1LUDMTG3qU_su9cJLHkTs-3ke6XgP62Lh/view?usp=sharing">
    <img src="https://drive.google.com/uc?export=view&id=1uZR9VG9ZMVtvoKd_e9LSbCNrDbY4AIEf" alt="Video Demo Robot 6DOF">
  </a>
  <p>Robot simulated in Gazebo environment and controlled via GUI</p>
</div>

## System Requirements

| Component             | Version or Description                                      |
|-----------------------|-------------------------------------------------------------|
| Operating System      | Windows 10/11 for MATLAB, Ubuntu 20.04 (Focal Fossa) for ROS |
| ROS                   | Noetic Ninjemys                                             |
| MoveIt                | 1.1.9                                                       |
| Gazebo                | 11.x                                                        |
| MATLAB                | R2024a+ with ROS Toolbox                                    |
| Additional Software   | Git, Python 3.8+, Catkin tools                              |
| Recommended Hardware  | RAM ≥ 8GB (preferably 16GB), CPU ≥ 4 cores, GPU with OpenGL support |

## Installation

1. Install ROS Noetic

Please refer to the official guide at: https://wiki.ros.org/noetic/Installation/Ubuntu

Or see detailed instructions in Vietnamese at: https://robodev.blog/cai-dat-ros-noetic

2. Install MoveIt
```bash
sudo apt install ros-noetic-moveit
```
3. Create ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

4. Clone Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/ThieuChiCong1048596/Mo-phong-robot-6-bac-tu-do.git
cd ../
catkin_make
source devel/setup.bash
```

5. Install MATLAB

* Install version R2024a or later
* Install ROS Toolbox via Add-Ons
* Verify connection:
```bash
rosinit('http://<ubuntu-ip>:11311')
```

## Usage Instructions

### 1. Start Simulation

roslaunch moveit_ar2_sim full_ar2_sim.launch

### 2. Run ROS Nodes

| Function                   | Command                                                   |
|----------------------------|-----------------------------------------------------------|
| End-Effector Tracking      | rosrun moveit_ar2_sim EE_tracker.py                      |
| IK Planning                | rosrun moveit_ar2_sim IK_solver.py                       |
| Cartesian Path Interpolation| rosrun moveit_ar2_sim Cartesian_path.py                 |
| Object Detection           | rosrun moveit_ar2_sim Detectobject.py                    |
| Position Calibration       | rosrun moveit_ar2_sim Calibrate.py                       |
| Spawn Objects in Gazebo    | rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py |
| Set Predefined Pose        | rosrun moveit_ar2_sim node_set_predefined_pose.py        |

### 3. MATLAB GUI Interface

rosinit('http://<ubuntu-ip>:11311')
run('Robot_6DOF_controller_GUI/gui.mlapp')

Or run the executable file:

./Robot_6DOF_controller_GUI/for_redistribution/MyAppInstaller_web.exe

## Project Structure

```plaintext
Mo-phong-robot-6-bac-tu-do/
├── moveit_ws/
│   └── src/
│       ├── moveit_ar2_sim/
│       │   ├── scripts/
│       │   ├── launch/
│       │   ├── config/
│       ├── ar2_robot/
│       │   ├── urdf/
│       │   ├── meshes/
│       │   ├── launch/
│       │   ├── config/
├── Robot_6DOF_controller_GUI/
│   ├── gui.mlapp
│   ├── for_redistribution/
│   ├── for_testing/
├── .gitignore
├── .gitattributes
├── README.md
├── LICENSE
```

## Documentation & Design

* SolidWorks 3D Files (Extract on Desktop, requires SolidWorks 2022 or later): (https://drive.google.com/drive/folders/10BUfy0PVBtMiwYveavlsReMv1TZBFfdv?usp=drive_link)
* GUI Executable (.exe): Included in .zip

## Algorithms

| Feature             | Description                                |
|---------------------|--------------------------------------------|
| IK Solver           | Solves inverse kinematics using MoveIt     |
| Cartesian Path      | Linear interpolation in space              |
| Object Detection    | Detects objects via image processing       |
| End-Effector Track  | Tracks and adjusts position                |

## Notes

* MATLAB and ROS master must be on the same LAN or run on localhost
* Start simulation in order:
1. **MoveIt** – Launch planning and robot configuration  
2. **Gazebo Simulation** – Start the simulation environment  
3. **ROS Functional Nodes** – Run custom ROS nodes (e.g., IK, detection)  
4. **MATLAB GUI** – Connect and control via graphical interface  

⚠️ **Tip**:  
Watch out for common issues with:
- **Networking** (ROS master URI mismatch)
- **TF Tree Conflicts**
- **Image Topics** not published or subscribed correctly

## Author

**Name**: Thiều Chí Công  
**Email**: [thieuchicong1048596@gmail.com](mailto:thieuchicong1048596@gmail.com)  
**GitHub**: [ThieuChiCong1048596](https://github.com/ThieuChiCong1048596)

## License

This project is licensed under the **MIT License**.
