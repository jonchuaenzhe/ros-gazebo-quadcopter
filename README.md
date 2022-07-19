March 2021
# Autonomous Quadcopter Simulation in ROS Gazebo

The project implements an autonomous quadcopter operation in gazebo simulation to shuttle between a fixed target (yellow box) and a moving target (autonomous ground robot). In order to accurately simulate real life pose estimation, the ground truth position of the quadcopter was not used. Instead, an Extended Kalman Filter (EKF) algorithm takes in IMU, Compass and GPS data to estimate its position.

Below is a video of the simulation:

https://user-images.githubusercontent.com/59247141/179737320-9792479c-baf1-46c8-b209-33775c0bb24a.mp4

## Installation

Run the following code to install and build the project
```
cd ~/workspace
sudo chmod +x *.sh
catkin_make
```
### Modification

The key files are hosted in the folder "workspace/src/ee4308/ee4308_hector/scripts/".
1. hector_master.py contains the finite state machine implementation for control of the quadcopter's behaviour
2. hector_move.py contains the PID controller for the quadcopter's motion
3. hector_motion.py contains the EKF implementation to estimate the pose of the quadcopter from the sensor data

## Operation

```
Once in the workspace directory, initialize the World
```
./proj2_init.sh
```
In a separate terminal, run the simulation
```
./proj2.sh
```