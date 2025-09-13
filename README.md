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


1. Dummy Robot Simulation
The dummy_robot.py node acts as a simple simulator. It initializes the robot at the origin and publishes its state to the /odom topic while listening for commands on /cmd_vel.
<img width="2184" height="1547" alt="Dummy_robot_simulation" src="https://github.com/user-attachments/assets/8d672297-cd36-49e7-86df-937c3606ebea" />

2. Fake Sensor Data Publisher
A separate terminal publishes messages to the /scan topic to simulate the robot's Lidar sensor. This example shows a "clear path" scenario where no obstacles are detected.
<img width="2184" height="1571" alt="Fake_Sensor_Data_Publisher
" src="https://github.com/user-attachments/assets/498d4f0d-68ac-44e4-a91e-95db7fa72b37" />


3. DWA Planner in Action
With the /odom and /scan topics active, the dwa_planner_node.py runs successfully. It continuously receives sensor data, evaluates multiple trajectories, and publishes the optimal velocity command and its associated cost to the console.
<img width="2184" height="1571" alt="DWA_Planner" src="https://github.com/user-attachments/assets/81e88382-8f96-47e6-9a29-034bbe6bf08c" />

4. RViz Visualization
The planner's output is visualized in RViz. The green marker at the origin represents the best trajectory selected by the DWA algorithm, which is published on the /dwa_trajectories topic as a MarkerArray.
<img width="2221" height="1524" alt="Rviz" src="https://github.com/user-attachments/assets/c1acce0b-9223-477f-8d07-a2058c2672e9" />
