# learning-ros1--project1

A project for learning ROS 1, doing some fun and not so challenging project.

- Pablo Aguado (https://github.com/aguadopd)
- Nicolás Pereyra (https://github.com/Ednip7)

## Meta

### Goals

At least:

1. Make a working, shareable package, using best packaging practices.
  - We should get acquainted with ROS packaging system.
2. Practice usage of ROS tools and files. Everything from `rqt_gui` to `rosdep` and `CMakeLists.txt`.
3. Write our very own launch files to launch a lot of nodes or even other launch files.
4. Write some nodes that do stuff.
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


### Roadmap

A coarse roadmap. Maybe just a checklist.

- [ ] Better define the outline of the project. Should we use the house world or the racetrack? What to do inside?
- [ ] Define if we go with classical CV or some NN based approach. It may depend on the objectives of the robot. Perhaps
      a mix.

### Questions and answers

#### Can we define launch files that launch the system partially?

It's probable that we need more processing power to do simulation and computer vision at the same time. Can we use one
launch file to launch some parts in one computer and some other parts in another one?
