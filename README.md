# Multiple Nonholonomic Robots Motion Planning

This repository contains the code for the paper:
xxx

Authors:xxx


## 1. Software Requirements
* Ubuntu 18.04
* ROS Melodic
* Octomap
* Gurobi

## 2. Setup and Config
#### (1) Install ROS Melodic for Ubuntu 18.04
Follow [ROS Installation](http://wiki.ros.org/ROS/Installation).

#### (2) Install Gurobi Optimization solver
We use Gurobi to solve the optimization problem, following [Gurobi Download and License instructions](https://www.gurobi.com/downloads/). It's noted that Gurobi needs the license to run, you can get a license for free with an academic mail address. This helpful [blog](https://blog.csdn.net/tuck_frump/article/details/130991493) guides configuring and using the Gurobi library in a Cmake project on Ubuntu.

#### (3) Install dependencies
```
sudo apt-get install ros-melodic-octomap*
sudo apt-get install ros-melodic-dynamic-edt-3d
```
#### (4) Build
After pre-setting, you can clone this repository to your catkin workspace and catkin_make. A new workspace is recommended.
```
cd ${YOUR_WORKSPACE_PATH}/src
git clone https:xxxxxxxxxx
cd ../
catkin_make
source ~/${YOUR_WORKSPACE_PATH}/devel/setup.bash
```

## 3. Run Simulations
xxx
#### (1) Simulation Configuration
You can configure the simulation settings in the launch files.

robot_radius: the robot implemented in our simulation is considered a circle on the map, the size is determined by the radius.

robot_count: the number of robots in the simulation.

minimum_order: the order of optimization for the spatial path, in which "3" represents jerk and "4" represents snap.



#### (2) Running on specified environment

#### (3) Running on random environment


## 4. Acknowledgements
Our implementation is built on top of [Multi-robot Trajectory Planner](https://github.com/LIJUNCHENG001/multi_robot_traj_planner) and [Btraj](https://github.com/HKUST-Aerial-Robotics/Btraj), we are grateful for their outstanding work.

