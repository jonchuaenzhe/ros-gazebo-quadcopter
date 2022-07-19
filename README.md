March 2021
# Autonomous Quadcopter Simulation in ROS Gazebo

The project implements an autonomous quadcopter operation in gazebo simulation to shuttle between a fixed target (yellow box) and a moving target (autonomous ground bot). An Extended Kalman Filter algorithm that takes in IMU, Magnetic Compass and GPS information (from simulation) is used to estimate its position.

https://user-images.githubusercontent.com/59247141/179737320-9792479c-baf1-46c8-b209-33775c0bb24a.mp4

## Installation

Change directory to workspace folder
```
cd ~/workspace
```
Assign permissions
```
sudo chmod +x *.sh
```
Build the folder
```
catkin_make
```

## Operation

Change directory to workspace folder
```
cd ~/workspace
```
Initialize the World
```
./proj2_init.sh
```
In a separate terminal, run the simulation
```
./proj2.sh
```