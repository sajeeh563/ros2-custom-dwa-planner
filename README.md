# Custom DWA Local Planner in ROS2 Humble

This project is a from-scratch implementation of the Dynamic Window Approach (DWA) local planner for a simulated robot in ROS2 Humble, completed as part of a technical assignment.

## Objective

The goal was to implement a custom DWA local planner without using pre-existing libraries like `nav2_dwb_controller`. The implementation covers velocity sampling, trajectory prediction, cost evaluation, and command publishing.

## Features Implemented

-   [x] **Dynamic Window Sampling:** Samples linear and angular velocities based on the robot's current state and acceleration limits.
-   [x] **Trajectory Prediction:** Simulates future paths for each sampled velocity command.
-   [x] **Cost-Based Evaluation:** Selects the optimal trajectory using a cost function that considers:
    -   Distance to the goal.
    -   Clearance from obstacles.
    -   Smoothness (penalizing drastic velocity changes).
-   [x] **ROS2 Integration:**
    -   Subscribes to `/odom` and `/scan` topics.
    -   Publishes control commands to `/cmd_vel`.
    -   Publishes all candidate trajectories as a `MarkerArray` to `/dwa_trajectories` for visualization in RViz.
-   [x] **Obstacle Avoidance:** Successfully generates avoidance maneuvers when obstacles are detected in the laser scan.
-   [x] **Meaningful Logging:** Provides real-time console logs of the selected velocity and its associated cost.

## Setup & Execution Instructions

This project uses Docker to provide a consistent and reproducible ROS2 Humble environment.

### 1. Build the Docker Image

The `Dockerfile` is included in the root of this submission. To build the image, run:
```bash
docker build -t ros2-dwa-dev .
