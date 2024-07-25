```markdown
# Environment Mapping with Swarm Robotics Using Single-Directional Ultrasonic Sensors

## Overview

This project demonstrates the integration of multiple ROS 2 packages to create an environment mapping system using swarm robotics equipped with single-directional ultrasonic sensors. The system includes ultrasonic sensor data processing, LaserScan data publishing, robot state management, wheel control, SLAM (Simultaneous Localization and Mapping), and navigation.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Conda (for activating the ROS 2 environment)

## Installation

### 1. Install Ubuntu

Ensure you have Ubuntu installed on your system. This project is tested with Ubuntu 22.04.

### 2. Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html).

### 3. Install Required Packages

If any package is missing, install it using:

```sh
sudo apt-get install <package_name>
```

Replace `<package_name>` with the name of the required package.

## Usage

### 1. Activate ROS 2 Environment

Open a terminal and activate the ROS 2 environment:

```sh
conda activate ros2
```

### 2. Run the System

**Run the following commands in separate terminals:**

- **Run the ultrasonic node:**

  ```sh
  ros2 run my_robot_ultrasonic ultrasonic_node
  ```

- **Run the LaserScan publisher:**

  ```sh
  ros2 run my_robot_ultrasonic ultrasonic_to_laserscan_node
  ```

- **Launch the URDF:**

  ```sh
  ros2 launch my_robot_description rsp.launch.py
  ```

- **Launch the TF (Transform) publisher:**

  ```sh
  ros2 run my_robot_state robot_state_publisher
  ```

- **Run the wheel node:**

  ```sh
  ros2 run my_robot_controller wheel_node
  ```

- **Activate keyboard control:**

  ```sh
  ros2 run my_robot_controller keyboard_control
  ```

- **Launch the SLAM Toolbox:**

  ```sh
  ros2 launch my_robot_mapping online_async.launch.py
  ```

- **Launch the Nav2 (Navigation) stack:**

  ```sh
  ros2 launch nav2_bringup navigation_launch.py
  ```

- **Run RViz2 with the default Nav2 configuration:**

  ```sh
  ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
  ```
