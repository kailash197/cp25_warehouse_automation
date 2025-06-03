# 📦 Robotics Developer Masterclass - Shelf Detector Final Project
*Warehouse Automation with RB-1 Robot*

## 🚀 Overview
This project implements an automated system for the RB-1 robot to:
- Navigation: Autonomously navigate a warehouse using **Nav2**.
- Perception: Detect and pick up a shelf using **lidar sensor**.
- Transport and release the shelf at a desired location.
- A **web interface** for monitoring/control.
- **Dockerized** deployment for scalability.

---

## 📋 Table of Contents
1. [Project Goals](#-project-goals)
2. [Prerequisites](#-prerequisites)
3. [Setup & Installation](#-setup--installation)
4. [Tasks Breakdown](#-tasks-breakdown)
 - [Task 1: Nav2 Setup](#task-1-nav2-setup)
 - [Task 2: Shelf Detection](#task-2-shelf-detection)
 - [Task 3: Web App & DevOps](#task-3-web-app--devops)
 - [Task 4: Documentation](#task-4-documentation)
5. [How to Run](#-how-to-run)
6. [Project Structure](#-project-structure)
7. [Results & Demo](#-results--demo)
8. [Troubleshooting](#-troubleshooting)
9. [Contributing](#-contributing)
10. [License](#-license)

---

## 🎯 Project Goals
- Implement **autonomous navigation** with Nav2.
- Develop **shelf detection/pickup** logic.
- Build a **user-friendly web dashboard**.
- Containerize the system using **Docker**.
- Document the process for reproducibility.

---

## ⚙️ Prerequisites
- **ROS 2 Humble/Humble**
- **Gazebo** (for simulation)
- **Nav2**
- **Docker** (for containerization)
- Python libraries: `rclpy`, `OpenCV`, `Flask`

---

## 📌 Tasks Breakdown

### **Task 1: Nav2 Setup**
- Configured Nav2 for RB-1 path planning.
- Localization using AMCL.
- Added collision avoidance for shelf transport.

### **Task 2: Shelf Detection**
- Used **LiDAR/vision** to detect shelf position.
- Implemented elevator control for pickup/release.
- Tested multiple start/goal locations.

### **Task 3: Web App & DevOps**
- **Flask-based dashboard** for monitoring robot status.
- Dockerized ROS 2 nodes and web app.

### **Task 4: Documentation**
- GitHub repo with setup instructions.
- Final presentation slides ([link](#)).

---
## 🛠️ Setup & Installation
### Simulation Setup
```bash
source ~/sim_ws/install/setup.bash
ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml
```

### Manual Robot Control (Test)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

---

## 🏃 How to Run
1. **Launch Simulation**:
```bash
source ~/sim_ws/install/setup.bash
ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml
```
2. **Start Nav2**:
```bash
ros2 launch rb1_navigation nav2_bringup.launch.py
```
3. **Run Shelf Detector Node**:
```bash
ros2 run shelf_detector main.py
```
4. **Web App**:
```bash
docker-compose up --build
```

## **Useful Commands**

1. Manual Robot Control (Test)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

2. Current position relative odom
```bash
ros2 topic echo /diffbot_base_controller/odom --field  pose.pose.position --once
```
3. View Frames
```bash
ros2 run tf2_tools view_frames
```

---

## 📂 Project Structure
```
warehouse_automation/
├── ros2_ws/ # ROS 2 workspace
├── web_app/ # Flask dashboard
├── docker/# Dockerfiles
├── docs/# Project documentation
└── README.md
```

---

## 📸 Results & Demo
![Gazebo Simulation](assets/gazebo_screenshot.png)
*Screenshot of RB-1 in Gazebo warehouse.*

[**Video Demo**](#) (Link to YouTube/GitHub video)

---

## ❗ Troubleshooting
- **Gazebo fails to launch**: Relaunch the simulation.
- **Nav2 localization issues**: Check `amcl` parameters.
- **Docker networking**: Ensure ROS 2 nodes can communicate.

---

## 🤝 Contributing
Pull requests welcome! Follow the [contribution guidelines](CONTRIBUTING.md).

---

## 📜 License
MIT License. See [LICENSE](LICENSE).

---

🔗 **Repository Link**: [GitHub Repo](#)
🎤 **Final Presentation**: [Google Slides](#)

---
