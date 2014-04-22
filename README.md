labrob
======

ROS Metapackage for the tutorials in http://www.romin.upm.es/wiki/

Developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the 
[Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the 
[Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). On going development continues in the master branch.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)
  
### Build Status

[![Build Status](https://travis-ci.org/fsuarez6/labrob.png?branch=master)](https://travis-ci.org/fsuarez6/labrob)

## Installation

### Basic Requirements

  1. Install [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Desktop Install** Recommended)
  2. Install [Gazebo 1.9](http://gazebosim.org/wiki/1.9/install)
```
sudo apt-get install ros-hydro-desktop gazebo
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
rosdep install --from-paths . --ignore-src --rosdistro hydro
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source ~/catkin_ws/devel/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the `.launch` file in the `labrob_gazebo` package:
```
roslaunch labrob_gazebo labrob_gazebo.launch
``` 

## Changelog

### 0.2.0 (2013-04-22)
* Added scoring interface (topics and services)
* Contributors: fsuarez6, ulikando

### 0.1.0 (2013-04-07)
* Initial Release
* Contributors: fsuarez6, ulikando
