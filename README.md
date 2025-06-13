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

