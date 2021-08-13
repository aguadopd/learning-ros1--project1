# learning-ros1--project1

A project for learning ROS 1, doing some fun and not so challenging project.

- Pablo Aguado (https://github.com/aguadopd)
- Nicolás Pereyra (https://github.com/Ednip7)


## Description

We simulate a Waffle Turtlebot 3 mobile robot inside a messy house. The robot will spawn in some arbitrary point and move around the house looking for some specific objects, informing their location. It uses the simulated camera feed to detect and recognize objects. The simulation ends when the robot has found all the required objects.


### STATUS

- *We simulate a Waffle Turtlebot 3 mobile robot inside a messy house.*
    - OK. Could have a larger variety of objects.
- *The robot will spawn in some arbitrary point and move around the house;*
    - No
- *looking for some specific objects;*
    - No.
- *informing their location.*
    - No.
- *It uses the simulated camera feed to detect and recognize objects.*
    - WIP.
- *The simulation ends when the robot has found all the required objects.*
    - No.


## Installation

*TODO*


## Usage

Just a manual launch for now.

1. In one terminal, start the Gazebo house simulation. **Should use it with the Waffle model, [as the Burger robot has no camera](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications)**.

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_house_custom.launch
```

2. In another terminal, launch the Darknet detection and classification node.
- Here we are passing an `image` option, to suscribe to the rgb image topic published by the Turtlebot3 simulator. This should open a new window with the detections.
- We are using the default launch file, that uses a Tiny YOLO v2 network that is automatically downloaded for testing while building the package. [This is trained on the COCO dataset, so should detect 80 different classes](https://github.com/leggedrobotics/darknet_ros).

```bash
roslaunch darknet_ros darknet_ros.launch image:=/camera/rgb/image_raw
```

3. Navigate manually launching a teleoperation node:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```


------------------------------------------------
------------------------------------------------


## Meta

### Goals

At least:

1. Make a working, shareable package, using best packaging practices.
  - We should get acquainted with ROS packaging system.
2. Practice usage of ROS tools and files. Everything from `rqt_gui` to `rosdep` and `CMakeLists.txt`.
3. Write our very own launch files to launch a lot of nodes or even other launch files.
4. Write some nodes that do stuff, probably with Python.
5. Use Gazebo and maybe modify some scenario.
5. Practice `git`. Probably try adding submodules.


### Constraints

- ROS 1 Noetic Ninjemys
- No hardware. Just simulation.


### Idea

Use a simulated Turtlebot robot inside a simulated world, adding Computer Vision capabilities to the robot, and then set
it to do some tasks inside that world.

- We already played with Turtlebot simulations when doing [Robotis' Turtlebot3
  tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). It seems that it's a good simulation
  and we could use that for something else.



### Roadmap, notes and etc

A coarse roadmap. Maybe just a checklist.

- [X] Better define the outline of the project. Should we use the house world or the racetrack? What to do inside?
  - Use house world
  - Place different objects around the use.
  - Give the robot a list of things to detect around the house.
	- There might be objects that the robot shouldn’t care about.
	- The robot should somehow inform the position where each object of interest was found.
- [X] Define if we go with classical CV or some NN based approach. It may depend on the objectives of the robot. Perhaps
      a mix.
  - We decided to use the [Darknet ROS](https://github.com/leggedrobotics/darknet_ros) that uses the Darknet implementation of YOLO detection and classification networks.
- [X] Create the base structure of folders. According to [this question](https://answers.ros.org/question/257855/git-strategy-for-catkin-and-package-folders/) our repo should be a folder that later can be put inside the `src` folder of some workspace.
  - [Darknet ROS](https://github.com/leggedrobotics/darknet_ros) and [Turtlebot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) will be submodules as we might want to modify them
- [X] Check if everything if we can use DarknetROS alongside the Turtlebot3 simulation.
- [ ] Try using a tiny YOLO model (v4) for better performance. (#4)
  - Models found in https://github.com/AlexeyAB/darknet#pre-trained-models
- [X] Waffle model is discontinued so maybe we should use Waffle Pi. Test if it works in the simulations. (#3)

MOVED TO GITHUB ISSUES


### Questions and answers

#### Can we define launch files that launch the system partially?

It's probable that we need more processing power to do simulation and computer vision at the same time. Can we use one
launch file to launch some parts in one computer and some other parts in another one?
