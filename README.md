# RL2025HW01 - Bring Up Your Robot

This repository contains the complete setup for the **Armando** 4-DOF manipulator, developed as part of the *Robotics Lab 2025 Homework 01*.  
The project demonstrates how to bring up a robot in **ROS 2**, visualize it in **RViz**, and simulate it in **Gazebo** using the **ros2_control** framework.

---

## 📦 Repository Packages

- **`armando_description`** — Contains the robot description (URDF/Xacro files), meshes, and configuration files.  
- **`armando_gazebo`** — Includes launch files and simulation setup for Gazebo.  
- **`armando_controller`** — Implements custom control nodes, launch files, and YAML configuration for controllers.

---

## ⚙️ Getting Started

To set up and test the project, follow these steps inside your ROS 2 workspace

1.  **Clone the Repository:**
```shell
cd ~/ros2_ws/src
git clone https://github.com/Federica2103/RL_2025_HW01.git
```
2.  **Build and Source:**
```shell
cd ..
colcon build
source install/setup.bash
```

## 💻 Usage Instructions
 ## **1. Launch the Manipulator in Rviz**

Start the robot visualization with the GUI for manual joint manipulation. To visualize the Armando manipulator in RViz2, open a terminal and execute:
```shell
ros2 launch armando_description armando_display.launch.py 
```
This will start RViz with the robot model loaded.
To visualize collision geometries (simplified as boxes), enable “Collision” under the RobotModel display options.

 ## **2. Launch the Manipulator in Gazebo**
To run the simulation in Gazebo, execute:
```shell
ros2 launch armando_gazebo armando_world.launch.py
```
The manipulator will appear in its default pose within the simulation environment.
Since gravity is active in Gazebo and no controllers are immediately active, the robot might droop until the controllers are started.

## **3.Visualize the Camera Sensor**
After launching Gazebo, open another terminal and run:
```shell
ros2 run rqt_image_view rqt_image_view
```
Select the correct topic (e.g., /videocamera) in the rqt_image_view tool.

## **4. Start Controller**
You can control position or trajectory of the robot by following commands:

```shell
ros2 launch armando_controller armando_control.launch.py controller_type:=<type>
```
Replace <type> with `position` or `trajectory` to launch the corresponding controller.

## Management/Monitoring Commands:
List Active Controllers:
 ```shell
    ros2 control list_controllers
 ```

Stop a Controller:
```shell
    ros2 control switch_controllers --stop <controller_name>
```

## **5. Subscriber and Publisher node**
A dedicated node, arm_controller_node, allows commanding the manipulator through a sequence of predefined positions.
```shell
ros2 run armando_controller arm_controller_node --ros-args -p use_trajectory:=<value>
```
where `<value>` is set to `false` to use the position controller or `true` to use the trajectory controller.
