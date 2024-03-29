# Reinforcement Learning experiment with Franka Emika robot manipulator.
## Experiment objective
This objective of the experiment is to teach the robot to correctly insert the object. The robot is the Franka Emika 7-DOF arm, it holds the cuboid in its gripper and the goal is to find the correct configuration to insert this cuboid inside the table.
There is a hole in the table that matches the size of the cuboid, but the hole is rotated by some angle. The robot must find the correct angle at which it can insert the cuboid.

![text](https://github.com/fenixkz/franka-rl/blob/master/Franka_qlearn.jpg)

## Prerequisites
The project has been implemented on Ubuntu 16.04 machine with ROS Kinetic. To run the project following libraries should be installed on your local machine:

  - Gazebo
  - MoveIt

## Running
```
$ roslaunch franka_gazebo panda_arm_hand.launch
$ cd franka_rl/src
$ python training.py
```

