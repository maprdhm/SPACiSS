# SPACiSS
Simulation of Pedestrians and an Autonomous Car in Shared Spaces

<img src=pedsim_simulator/images/pedsim_av.png width=500/>

The implementation is based on [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros), ROS packages that wrap a crowd simulator based on Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library.  
The pedestrian model is based on the social force model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf).

This package is useful to support Autonomous vehicle (AV) developments that require the simulation of pedestrians and an AV in various shared spaces scenarios.  
It allows:  
&nbsp;&nbsp;&nbsp;1. in simulation, to pre-test AV navigation algorithms in various crowd scenarios,  
&nbsp;&nbsp;&nbsp;2. in real crowds, to help online prediction of pedestrian trajectories around the AV.

<br/>

##  Contents
* **[Features](#features)**
* **[Installation](#installation)**
* **[AV control modes](#av-modes)**
* **[Launch files](#launch-files)**
* **[Scenarios](#scenarios)**
* **[Reference paper](#reference-paper)**
* **[License](#license)**
* **[Contributors](#contributors)**
* **[Acknowledgments](#acknowledgments)**

<br/>

## Features
- Heterogeneous crowds in shared spaces for simulation and prediction of 100 pedestrians in real time.
- Individuals and groups walking using social force model, with various social relationships
- Prius AV model (from https://github.com/osrf/car_demo/tree/master/prius_description) or Zoe AV model (from https://univ-nantes.io/hamon-a/icars-public), controlled from within the simulation or by an external ROS controller publishing velocity commands.
- Pedestrians reactions to the AV
- 3 shared space environments (business area, campus, citer center) and 8 scenarios for each env. ready to test AV navigation algorithms
- Sensors simulation (point clouds in AV frame for people and walls)
- XML based scene design
- Visualization using Rviz
- Optional plugin to connect with Gazebo

<br/>

## Installation and launch

### Requirements
(Tested on Ubuntu 16.04 with ROS Kinetic and Gazebo7 and on Ubuntu 18.04 with ROS Melodic and Gazebo9)
- ROS with the navigation and visualization stack
- C++11 compiler
- Qt4

### Install commands
```
cd [workspace]/src
git clone https://github.com/maprdhm/Spaciss.git  
cd Spaciss
git submodule update --init --recursive
cd ../..

catkin_make or catkin build (twice at the first time)
```

### Sample usage
```
roslaunch experimental_package business_area.launch
```
The previous command should start the simulator with the business area environment and the shared space scenario (many pedestrians and an AV).  
More launch files are in the `experimental_package/launch` repository.  
Ready to test scenarios are in the `experimental_package/scenarios` repository.  


### Parallel execution of multiple nodes (only available on Ubuntu 16 with ROS Kinetic)

```
roslaunch experimental_package multi_node_parallel.launch
```
The previous command starts 2 simulations with the business area environment and the shared space scenario, with AV max speed of 2m/s and 4m/s respectively.  


<br/>

## AV control modes

### Mode 0 or 1: with an external controller

The AV in the simulator is controlled externally using any controller than sends [Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages.

Example with the AV controlled by [move_base planner](http://wiki.ros.org/move_base):
```
roslaunch experimental_package business_area_external.launch
```
and to send goals to the planner:
```
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "odom"}, pose: {position: {x: -10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}'
```



### Mode 2: with the social force model

The AV in the simulator is controlled by the social force model, like for pedestrians.  
For now, the social force is configured to have the AV go straight to its destination, trying to avoid walls but not pedestrians.

<br/>

## Launch files

* <b> simulator node</b>
* visualizer node + rviz node
* sensor node 
* robot controller node and move_base node
* timer node to shutdown all nodes after a timelapse

### Parameters for the simulator node
#### Simulation

| Parameter  | Use | Default value | Possible values
| ------------- | ------------- | ------------- | ------------- |
| visualize | Rviz visualisation  | true  | boolean  |
| simulation_factor  | Simulation speed up  | 1.0 | double  |
| update_rate  | Simuation step (Hz)  | 25.0 (0.04 s) | double  |
| scene_file  | Scenario  | "$(find experimental_package)scenarios/business_area/shared_space.xml" | absolute path to xml scenario file |
| default_queue_size  | size of queue for published msgs | 10 | int |


#### AV

| Parameter  | Use | Default value | Possible values
| ------------- | ------------- | ------------- | ------------- |
| kbd_teleop | Keyboard control of AV  | false  | boolean  |
| rqt_teleop  | ROS rqt control of the AV (UI)  | false | boolean  |
| with_robot  | Simulation with an AV  | true  | boolean  |
| pose_initial_x  | x postion of the AV  | 5.0  | double  |
| pose_initial_y  | y postion of the AV  | 5.0  | double  |
| pose_initial_theta  | Orientation of the AV  | 0.0  | double |
| robot_description  | URDF model of AV  | prius.urdf | absolute path to urdf file |
| max_robot_speed  | Max speed of AV (m/s)  | 3.0 | double |
| robot_mode  | AV control mode | 1 | 0 or 1: teleoperation / 2: social_drive (with SFM) |

#### Pedestrians

| Parameter  | Use | Default value | Possible values
| ------------- | ------------- | ------------- | ------------- |
| enable_distraction | Agent visual distraction | false  | boolean  |
| probability_random_stop | Agent probability to do a temporary stop | 0.0  | double [0..1]  |

#### Groups

| Parameter  | Use | Default value | Possible values
| ------------- | ------------- | ------------- | ------------- |
| enable_groups | Groups presence  | true  | boolean  |
| group_size_lambda  | Lambda parameter in Poisson law distribution for groups sizes  | 1.1 | double>0 |
| groups_couples_proportion  | Proportion of couples in groups  | 0.0 | double [0..1] Couples can only be groups of 2. If proportion(group_size=2) &lt; groups_couples_proportion -> generated groups_couples_proportion &lt; groups_couples_proportion  |
| groups_friends_proportion  | Proportion of couples in groups  | 1.0 | double [0..1]  |
| groups_families_proportion  | Proportion of couples in groups  | 0.0 | double [0..1]  |
| groups_coworkers_proportion  | Proportion of couples in groups  | 0.0 | double [0..1]  |

<!--
#### Forces
| Parameter  | Use | Default value | Possible values
| ------------- | ------------- | ------------- | ------------- |
| force_random | Factor random force  | 0.1  | double  |
| force_obstacle | Factor obstacle force  | 10.0  | double  |
| sigma_obstacle | To scale obstacle force  | 0.2  | double  |
| force_social | Factor social force  | 5.1  | double  |
| force_wall | Factor allong wall force. If agent is stuck: move along the wall. Now managed in obstacleForce. | 2.0  | double  |
| force_group_gaze | Factor group_gaze force  | 3.0  | double  |
| force_group_coherence | Factor group_coherence force. Changed to get more cohesive agents (initial default value: 2.0)  | 20.0 | double  |
| force_group_repulsion | Facto group repulsion force to prevent from overlapping. Disabled, managed in social force.  | 0.0  | double  |
-->

<br/>

## Scenarios

     <scenario>
        <waypoint id="wu" x="0" y="-20" r="2" b="2"/>
        <waypoint id="wd" x="0" y=" 20" r="2" />
        <obstacle x1="10" y1="30" x2="10" y2="-30"/>
        <agent x="0" y="-30" n="12" dx="10" dy="10" type="0" purpose="1">
            <addwaypoint id="wd" />
            <addwaypoint id="wu" />
        </agent>
    </scenario>

### waypoint
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| id | Identifiant  | string
| x  | x position  | double
| y  | y position  | double
| r (opt, default 0) | radius  | double
| b (opt, default 0) | behaviour  | 0: simple / 1: source / 2: sink

### obstacle
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| x1  | x1 position: obstacle start  | double
| y1  | y1 position: obstacle start  | double
| x2  | x2 position: obstacle end  | double
| y2  | y2 position: obstacle end  | double

### agent
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| x  | x initial position | double
| y  | y initial position | double
| n  |  Number of agents, to change crowd density | int
| dx (opt if n=1) | Dispersion of agents along x axis  | double
| dy (opt if n=1) | Dispersion of agents along y axis  | double
| type (opt, default 0) | Agent type (affects the max speed and force desired for elderly, affects the max speed/size/max rotation angle/force social and force obstacle for robot)  | 0: adult / 1: child (unused) / 2: robot (=AV) / 3: elder / 4: immob (stationary)
| purpose (opt, default 0) | Trip purpose of agent (affects the max speed except for robot and elderly)  | 0: unknown / 1: work / 2: leisure
| addwaypoint | id  | id of a waypoint

### source (spawn area)
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| x  | x initial position | double
| y  | y initial position | double
| n  |  Number of agents | int
| dx (opt if n=1) | Dispersion of agents along x axis  | double
| dy (opt if n=1) | Dispersion of agents along y axis  | double
| type (opt, default 0) | Agent type | 0: adult / 1: child (unused) / 2: robot (=AV) / 3: elder / 4: immob (stationary)
| purpose (opt, default 0) | Trip purpose of agent | 0: unknown / 1: work / 2: leisure
| addwaypoint | id  | id of a waypoint

### attraction
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| id | Identifiant  | string
| x  | x initial position | double
| y  | y initial position | double
| width  |  Width of attraction zone | double
| height  |  Height of attraction zone | double
| strength | Attraction strength  | double

<br/>


## Reference paper
Manon Prédhumeau. 2021. Simulating Realistic Pedestrian Behaviors in the Context of Autonomous Vehicles in Shared Spaces: Doctoral Consortium. In Proc. of the 20th International Conference on Autonomous Agents and Multiagent Systems (AAMAS 2021), May 3–7, 2021, IFAAMAS, 3 pages.
[http://www.ifaamas.org/Proceedings/aamas2021/pdfs/p1829.pdf](http://www.ifaamas.org/Proceedings/aamas2021/pdfs/p1829.pdf)


## Licence
The core `libpedsim` is licensed under LGPL.  
The ROS integration and extensions are licensed under BSD.


## Contributors
Manon Prédhumeau,
Lyuba Mancheva,
Julie Dugdale,
Anne Spalanzani

The package is a **work in progress** mainly used in research prototyping.


## Acknowledgements
This work has been developed as part of the [HIANIC](https://project.inria.fr/hianic/) project.

