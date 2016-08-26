labrob
======

ROS Metapackage for the tutorials in http://www.romin.upm.es/wiki/

**Maintainer:** [Francisco Suárez Ruiz](http://fsuarez6.github.io/)

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)
  
### Build Status

[![Build Status](https://travis-ci.org/fsuarez6/labrob.png?branch=master)](https://travis-ci.org/fsuarez6/labrob)

## Installation

### Basic Requirements

  1. Install [ROS Kinetic](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Base Install** Recommended)
```
sudo apt-get install ros-$ROS_DISTRO-base
``` 

### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/fsuarez6/labrob/master/labrob.rosinstall
wstool update
``` 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file:
```
source ~/catkin_ws/devel/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try to control the robot using the keyboard:
```
roslaunch labrob_worlds crazy_maze.launch
rosrun labrob_control key_teleop.py
``` 

## Changelog

### 0.3.0 (2016-08-26)
* Made it compatible with ROS kinetic
* Contributors: fsuarez6

### 0.2.0 (2014-04-22)
* Added scoring interface (topics and services)
* Contributors: fsuarez6, ulikando

### 0.1.0 (2014-04-07)
* Initial Release
* Contributors: fsuarez6, ulikando
