<div align ="center">
<h3> ICRA 2025: Multi-Nonholonomic Robot Object Transportation with Obstacle Crossing using a Deformable Sheet </h3>

Weijian Zhang, Charlie Street, Masoumeh Mansouri

University of Birmingham

<a href="https://ieeexplore.ieee.org/document/11128313"><img alt="Paper" src="https://img.shields.io/badge/Paper-IEEE%20Xplore-pink"/></a>
<a href="https://youtu.be/DvEniLKDaH0"><img alt="Video" src="https://img.shields.io/badge/Video-Youtube-red"/></a>
</div>

## Overview
<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/prob_overview.png" alt="formation_planning" width="734" height=400">
</p>

We address multi-robot formation planning where nonholonomic robots collaboratively transport objects using a deformable sheet in unstructured, cluttered environments.
Our paper has been selected as the <span style="color:red; font-weight:bold;">Best Conference Paper Award Finalist & Best Paper Award Finalist on Multi-Robot Systems at ICRA 2025</span>!
<!-- <table style="width:100%; text-align:center;">
  <tr>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo1%20(1).png" alt="Image 1" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo2%20(1).png" alt="Image 2" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo3%20(1).png" alt="Image 3" width="352" height="200"></td>
  </tr>
  <tr>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo4%20(1).png" alt="Image 4" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo5%20(1).png" alt="Image 5" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo6%20(1).png" alt="Image 6" width="352" height="200"></td>
  </tr>
</table> -->
<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/video.gif" alt="Multi-Formation Planning and Coordination for Object Transportation" width="600">
</p>

## Features

<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/system_overview.png" alt="formation_planning" width="754.9" height=400">
</p>

 - A heuristic path exploration method that efficiently evaluates a set of homotopically distinct solution spaces for the formation.
 
 - A two-stage iterative motion planning framework for finding locally time-optimal collision-free formation trajectories using a deformable sheet.

## Requirements

 - ROS Noetic or later
 - Ubuntu 20.04 or later
 - yaml-cpp 0.8.0 or later
 - You'll also need a license for the Mosek optimization toolbox <https://www.mosek.com/> (this package includes a downloader for the Mosek code, but you have to get your own license). Mosek has free licenses available for academic use.

## Installation

1. Create a new workspace:

```shell
$ mkdir -p ~/CPDOT/src
$ cd ~/CPDOT/src
$ catkin_init_workspace
```

2. Clone the package into the workspace:

```shell
$ git clone git@github.com:HyPAIR/CPDOT.git
```

3. Install dependencies:
```shell
$ cd ~/CPDOT
$ rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace (Set MOSEK_DIR to the root path of your __MOSEK license__ (e.g., _/home/yourname/mosek/7_):

```shell

$ catkin_make --cmake-args -DMOSEK_DIR=/home/yourname/mosek/7
```

## Parameter values in the simulation
| **Parameter**                                | **Value**         | **Description**                          |
|----------------------------------------------|-------------------|------------------------------------------|
| $L$                                          | $0.65m$           | Car-like robot wheelbase                 |
| $v_{m}^{car}$ | $1.0m/s$ | Linear velocity limit                    |
| $a_{m}^{car}$                 | $1.0m/s^2$        | Linear acceleration limit                |
| $\phi_{m}^{car}$                             | $0.68rad$         | Steering angle limit                     |
| $\omega_{m}^{car}$       | $0.2rad/s$ | Angular velocity limit                    |
| $\Omega_{m}^{car}$                           | $2.5rad/s^2$      | Angular acceleration limit               |
| $-$                                          | $0.2m$            | IRIS grid size                           |
| $\Delta{t}$                                  | $0.15s$           | Time between control inputs              |
| $\mathbf{W}$                                 | $[2, 0; 0, 1]$ | Weights for cost function |
| $l^{max}_i$                                  | $2.0m$            | Original sheet's side length             |
| $z_r$                                        | $1.5m$            | Height of each contact point             |
| $d_0$                                        | $1.5m$            | Default inter-robot distance             |
| $\delta$                                     | $0.2m$            | Delta value on inter-robot distance      |
| $\lambda_1$                                  | $1.0$            | Weight parameter                          |
| $\lambda_2$                                  | $1.0$            | Weight parameter                          |
| $\lambda_3$                                  | $1.0$            | Weight parameter                          |
| $\lambda_4$                                  | $1.0$            | Weight parameter                          |

## Test in Rviz

Launch the simulation to the trajectory optimisation result (4 robots in a simple scenario):

  ```shell
  $ roslaunch formation_planner topological_test.launch
  ```

<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/icra_4.png" alt="task_allocation" width="862" height="400">
</p>

## Test in Gazebo

Create a world file with 5 car-like robots in 100 random obstacle environments.

  ```shell
  $ roslaunch formation_planner write_obs_to_world.launch
  ```

Launch a multi-robot transportation simulation, with 5 car-like robots in 100 random obstacle environments.

  ```shell
  $ roslaunch formation_planner heterogeneous_triangle.launch
  ```

Launch the control node:

  ```shell
  $ roslaunch formation_planner control_triangular_formation.launch
  ```

<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/snapshot_100.png" alt="fg_10_real" width="685" height=400">
</p>


## Video

A simulation and real-world experiments video demonstrating our proposed framework can be found at [bilibili](https://www.bilibili.com/video/BV1BV4sesEKX/?spm_id_from=333.1387.list.card_archive.click&vd_source=bf49c74265570abfae0e3bacc588f839)/[youtube](https://www.youtube.com/watch?v=DvEniLKDaH0&t=2s).

### Citation

If you find this work useful, please cite ([paper](https://ieeexplore.ieee.org/document/11128313)):

```bibtex
@INPROCEEDINGS{11128313,
  author={Zhang, Weijian and Street, Charlie and Mansouri, Masoumeh},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Multi-Nonholonomic Robot Object Transportation with Obstacle Crossing Using a Deformable Sheet}, 
  year={2025},
  volume={},
  number={},
  pages={7349-7355},
  keywords={Limiting;Navigation;Transportation;Probabilistic logic;Hardware;Planning;Iterative methods;Robots;Trajectory optimization;Contracts},
  doi={10.1109/ICRA55743.2025.11128313}}
```
