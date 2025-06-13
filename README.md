# Deployment of Dynamic Obstacle Avoidance System Optimizing D*-Lite, A*, and Dijkstra on TurtleBot

This repository contains the implementation of a dynamic obstacle avoidance system for TurtleBot, optimizing D*-Lite, A*, Dijkstra, and Rapidly-exploring Random Tree (RRT) algorithms for autonomous navigation. The system is designed to handle dynamic obstacles in real-time by recalculating paths and adjusting for changes in the environment using simulations in Gazebo and Rviz.

## Table of Contents
1. [Introduction](#introduction)
2. [Installation Instructions](#installation-instructions)
3. [How to Run the Project](#how-to-run-the-project)
4. [Demo Video](#demo-video)
5. [Algorithms Used](#algorithms-used)
6. [Results and Comparison](#results-and-comparison)
7. [License](#license)

## Introduction

This project implements and compares the performance of four path planning algorithms — D*-Lite, A*, Dijkstra, and RRT — for dynamic obstacle avoidance in autonomous robots. The unique contribution of this work is the introduction of a custom cost function for D*-Lite, balancing robot safety (distance from obstacles) and the time to reach the goal. 

The system was tested in a Gazebo simulation environment with a TurtleBot robot, showcasing the real-time adaptation to dynamic obstacles and comparing the performance of these algorithms in terms of safety, path efficiency, and computational cost.

## Installation Instructions

To run this project, you'll need to have the following software installed:

- [ROS (Robot Operating System)](https://www.ros.org/)
- [Gazebo](http://gazebosim.org/)
- [Rviz](http://wiki.ros.org/rviz)

### Steps to Set Up the Project

1. **Clone the Repository**
   Clone this repository to your local machine using the following command:

   ```bash
   git clone https://github.com/PhanindratejaThammi/Deployment-of-Dynamic-Obstacle-Avoidance-System-optimizing-Dstar-Lite-Astar-Dijkstra-on-TurtleBot.git
   cd Deployment-of-Dynamic-Obstacle-Avoidance-System-optimizing-Dstar-Lite-Astar-Dijkstra-on-TurtleBot

2. **Create a Workspace**
   If you don't have an existing ROS workspace, create one:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   
3. **Copy the `src` Folder**
   Copy the `src` folder from this repository into your workspace:
   ```bash   
   cp -r /path/to/Deployment-of-Dynamic-Obstacle-Avoidance-System-optimizing-Dstar-Lite-Astar-Dijkstra-on-TurtleBot/src ~/catkin_ws/src/

4. **Build the Package**
   Navigate to your workspace directory and build the package with catkin_make:
   ```bash   
   cd ~/catkin_ws
   catkin_make
   
5. **Source the Workspace**
   After the build completes, source the workspace:
    ```bash
    source devel/setup.bash

6. **Launch the Simulation**
   Launch the Gazebo simulation and the TurtleBot setup using the following commands:

   - **(a) To launch the world with the TurtleBot:**
     ```bash
     roslaunch ros_world Turtlebot3_world_RL.launch
     ```

   - **(b) To launch SLAM (Simultaneous Localization and Mapping) for obstacle detection:**
     ```bash
     roslaunch turtlebot3_slam turtlebot3_gmapping_RL.launch
     ```

   - **(c) To run the path planning algorithm:**
     ```bash
     python3 path_planning.py
     ```

7. **Provide Goal to the TurtleBot**
   After running the above commands, use Rviz to set the goal point for the robot. Click on the "2D Nav Goal" button in Rviz to set the destination for the robot.

## Demo Video
https://github.com/user-attachments/assets/00c1170a-caef-48de-b926-252c2a6041dd

## Algorithms Used
1. **D-star-Lite**: An incremental path planning algorithm designed for dynamic environments. It recalculates the path based on local changes, making it more efficient than A* and Dijkstra in dynamic settings.

2. **A-Star**: A classic search algorithm that finds the shortest path from a start node to a goal node. It is not optimal in dynamic environments since it recalculates the path from scratch when obstacles appear.

3. **Dijkstra**: A well-known algorithm for finding the shortest paths from a source node to all other nodes in a graph. Similar to A*, it does not handle dynamic obstacles well.

4. **RRT (Rapidly-exploring Random Tree)**: A path planning algorithm used for complex, high-dimensional spaces. It is used for fast exploration of feasible paths in unknown environments.

## Results and Comparison
The performance of D*-Lite, A*, and Dijkstra algorithms was compared in the Gazebo simulation. The D*-Lite algorithm showed significant improvements in terms of path replanning efficiency. Key results from the comparison include:

1. **Path Length**: D*-Lite provides nearly the same path length as A* and Dijkstra when the goal is close, but shows more optimized path lengths for distant goals.

2. **Replanning Speed**: D*-Lite is faster in replanning, as it reuses previously computed data and only updates the affected areas.

3. **Efficiency**: D*-Lite outperforms both A* and Dijkstra in dynamic environments, showing fewer nodes in the open list and faster path updates.

   



