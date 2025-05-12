# Yahboom MicroROS-Pi5 Custom ROS 2 Nodes

This repository contains custom ROS 2 nodes and Docker startup scripts for the Yahboom Robot Car using Raspberry Pi 5 and Micro-ROS with ROS 2 Humble. It extends the original Yahboom setup with additional capabilities such as obstacle detection, camera streaming, line following, and hand gesture-based control.

## Features

- **Drive Node**
  - Controls robot motion.
  - Automatically stops the car upon detecting an obstacle via LiDAR.

- **Camera Node**
  - Publishes image frames from the onboard camera to a ROS 2 topic.

- **Follow Line Node**
  - Uses HSV-based color tracking and a PID controller to follow a colored line.

- **Hand Control Node**
  - Detects and interprets hand gestures using MediaPipe to command the robot.

## Startup Scripts

- `custom_ros2_humble.sh`  
  Docker script to start the custom container with appropriate privileges and mounts.

- `startup.sh`  
  Script executed within the container to launch all custom ROS 2 nodes.

---

## Setup Instructions

### 1. Clone This Repository

```bash
git clone https://github.com/yourusername/yahboom-microros-pi5.git
cd yahboom-microros-pi5
```

### 2. overwrite the original ros2_humble.sh with the custom_ros2_humble.sh

## Orignal Source code : https://drive.google.com/drive/folders/16n1kfoHGunD2CuZ_Je-8tCHgshOiDSGN
