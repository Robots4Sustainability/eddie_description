# Freddy Description

Robot description for the freddy robot in form of URDF files, controllers and meshes.
Gripper not added.

## Environment
Tested on:
  Ubuntu: 22.04
  ROS2: Humble

## Setup

- Clone this repository into your workspace
  
  ```bash
  # Create workspace
  mkdir -p ~/freddy_ws/src && cd ~/freddy_ws/src

  # Clone repository
  git clone https://github.com/a2s-institute/freddy_description.git -b gz-devel
  ```

- Build workspace

  ```bash
  cd ~/freddy_ws

  colcon build
  ```

## Usage

- View robot in rviz

  ```bash
  cd ~/freddy_ws

  # Source workspace
  source install/setup.bash

  # View robot in rviz
  ros2 launch freddy_description view_freddy.launch.py joint_state_gui:=false
  ```

- View robot in rviz with joint state gui

  ```bash
  ros2 launch freddy_description view_freddy.launch.py joint_state_gui:=true
  ```

## Freddy

![Freddy](media/freddy_default_rviz.png)
