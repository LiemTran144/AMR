# Autonomous Mobile Robot (AMR) Development using ROS 2

**Author:** Tran Trung Liem  
**GitHub:** [LiemTran144/AMR](https://github.com/LiemTran144/AMR)

This repository hosts the complete source code for a full-stack Autonomous Mobile Robot (AMR) project. The development journey spans from building the **Low-level Hardware Abstraction Layer** (Drivers, Telemetry, Control) to deploying **High-level Autonomous Navigation** (A* Path Planning, DWA, SLAM) on an NVIDIA Jetson Nano.

## 📺 Demo & Results

### 1. Control & Telemetry System (Foundation Project)
*Showcasing the custom PyQt Dashboard for real-time monitoring, database logging, manual control, and waypoints tracking via Modbus RTU.*

[![Control System Demo](https://img.youtube.com/vi/QExMpKI5Sls/0.jpg)](https://youtu.be/QExMpKI5Sls)

### 2. Advanced Navigation (Capstone Project)
*Demonstrating Global Path Planning with a custom A* Plugin, Local Trajectory Optimization with DWA, and Dynamic Obstacle Avoidance.*

[![Control System Demo](https://img.youtube.com/vi/Si51abrtMTA/0.jpg)](https://youtu.be/Si51abrtMTA)

---

## 🚀 Project Overview

This repository represents a continuous development process divided into two major phases:

### Phase 1: Infrastructure & Telemetry (The Foundation)
*Goal: Establish a robust hardware-software bridge and operational data logging.*
* **Hardware Interface:** Developed a custom **Modbus RTU driver** to communicate with ZLAC8015D Servo Motor Drivers, bridging the gap between embedded hardware and ROS 2 nodes.
* **Control System:** Implemented a **PD Controller** with "rotate-to-align" logic (Rotation Shim equivalent) for precise waypoint tracking.
* **Monitoring GUI:** Engineered a **Qt-based interface (PyQt)** integrated with **MySQL**. This allows for real-time logging of velocity, errors, and odometry data for kinematic validation and debugging.

### Phase 2: Autonomous Navigation (The Intelligence)
*Goal: Implement intelligent path planning and dynamic obstacle avoidance.*
* **Custom A* Planner:** Developed a custom **A* Global Planner plugin** for the Nav2 framework to optimize pathfinding efficiency on embedded hardware.
* **DWA Controller:** Engineered a custom **Dynamic Window Approach (DWA) plugin** to handle local trajectory planning and smooth velocity control.
* **Perception:** Configured **Costmap 2D** with inflation layers and integrated **SLAM Toolbox** & **AMCL** for robust localization using LiDAR data.

---

## 🛠️ Technologies & Hardware

### Software Stack
* **OS:** Ubuntu 22.04 LTS
* **Framework:** ROS 2 Humble Hawksbill
* **Navigation:** Nav2 Stack (Custom Plugins), SLAM Toolbox
* **Languages:** C++ (Plugins/Core), Python (Nodes/GUI), SQL (Database)
* **Tools:** Qt Designer, MySQL Workbench, RViz2, Gazebo

### Hardware Specs
* **Computer:** NVIDIA Jetson Nano
* **Sensors:** RPLidar A1 || OLEI 2D LR 1BS2
* **Actuators:** ZLAC8015D Hub Servo Motors
* **Chassis:** Differential Drive Configuration

---

## ⚙️ Installation & Build

### Prerequisites
* ROS 2 Humble installed on Ubuntu 22.04.
* `colcon` build tool.
* MySQL Server (for the telemetry module).

### Build Steps

1.  **Clone the repository:**
    ```bash
    mkdir -p ~/amr_ws/src
    cd ~/amr_ws/src
    git clone [https://github.com/LiemTran144/AMR.git](https://github.com/LiemTran144/AMR.git)
    ```

2.  **Install dependencies:**
    ```bash
    cd ~/amr_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```

4.  **Source the setup:**
    ```bash
    source install/setup.bash
    ```

---

## 🤝 Acknowledgements & Credits

This project was developed as part of my engineering coursework at **Eastern International University (EIU)**.

* **Base Framework:** The core architecture and initial setup were adapted from the foundational codebase provided by my mentor, **Mr. Tran Duy Nhat** ([@NhatTran-97](https://github.com/NhatTran-97)).
* **Extensions & Contributions:** I significantly extended the base framework by:
    * Developing custom **Nav2 Plugins** (A* & DWA) from scratch in C++.
    * Integrating the **MySQL Database** and developing the **PyQt Control Dashboard**.
    * Implementing the **Modbus RTU Driver** for ZLAC8015D integration.
* **Open Source:** Special thanks to the ROS 2 and Navigation 2 communities.

---

**Contact:** Tran Trung Liem  
**LinkedIn:** [linkedin.com/in/liem-tran2003](https://www.linkedin.com/in/liem-tran2003)  
**Email:** liem.trantrung2003@gmail.com
