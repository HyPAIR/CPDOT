# Multi-Nonhonolomic Robot Object Transportation with Obstacle Crossing using a Deformable Sheet

We address multi-robot formation planning where nonholonomic robots collaboratively transport
objects using a deformable sheet in unstructured, cluttered environments.
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
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/prob_overview.png" alt="formation_planning" width="734" height=400">
</p>
<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/video.gif" alt="Multi-Formation Planning and Coordination for Object Transportation" width="600">
</p>

## Features

 - A heuristic path exploration method which efficiently evaluates a set of homotopically distinct solution spaces for the formation.
 
 - A two-stage iterative motion planning framework for finding locally time-optimal collision-free formation trajectories using a deformable sheet.

## Requirements

 - ROS Noetic or later
 - Ubuntu 20.04 or later
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
$ rosdep install formation_planner
```

4. Build the workspace:

```shell
$ cd ~/CPDOT
$ catkin_make
```

## Parameter values in the simluation
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
## Test in Rviz

Launch the simulation to trajectory optimisation result (4 robots in a simple scenario):

  ```shell
  $ roslaunch formation_planner topological_test.launch
  ```

<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/icra_4.png" alt="task_allocation" width="862" height="400">
</p>

## Test in Gazebo

Launch multi-robot transportation simulation, with 4 car-like robots in 100 random obstacle environments.

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

A simulation and real-world experiments video demonstrating our proposed framework can be found at [youtube](https://youtu.be/7gs06rqIHbM).
<!-- ## Citation

If you find this work useful, please cite [A decoupled solution to heterogeneous multi-formation planning and coordination for object transportation](https://www.sciencedirect.com/science/article/pii/S092188902400157X) ([pdf](http://SimonZhang1999.github.io/files/ras_2024.pdf)):

```bibtex
@article{zhang2024decoupled,
  title={A decoupled solution to heterogeneous multi-formation planning and coordination for object transportation},
  author={Zhang, Weijian and Street, Charlie and Mansouri, Masoumeh},
  journal={Robotics and Autonomous Systems},
  pages={104773},
  year={2024},
  publisher={Elsevier}
} -->
