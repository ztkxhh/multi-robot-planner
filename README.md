# Multiple Nonholonomic Robots Motion Planning

This repository contains the code for the paper:
xxx

Authors:xxx


## 1. Software Requirements
* Ubuntu 18.04
* ROS Melodic
* Octomap
* Gurobi

## 2. Installation instructions
#### (1) Install ROS Melodic for Ubuntu 18.04
Follow [ROS Installation](http://wiki.ros.org/ROS/Installation)

#### (2) Install Gurobi Optimization solver
We use Gurobi to solve the optimization problem, following [Gurobi Download and License instructions](https://www.gurobi.com/downloads/). It's noted that Gurobi needs the license to run, you can get a license for free with an academic mail address. This helpful [blog](https://blog.csdn.net/tuck_frump/article/details/130991493) guides configuring and using the Gurobi library in a Cmake project on Ubuntu.

#### (3) Install dependencies
```
sudo apt-get install ros-melodic-octomap*
sudo apt-get install ros-melodic-dynamic-edt-3d
