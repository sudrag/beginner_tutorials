# Beginner Tutorials for ROS 

## Overview

This project is an introduction to ROS and performs beginner tutorials and executes a simple publisher and subscriber implemented in C++. The project involves understanding of ROS topics, nodes, messages and services. The publisher node publishes a modified custom string which is then heard by the subscriber. 

## To Build

* Creating a catkin workspace:

```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
```
* Cloning the repository and build:
```
cd catkin_ws
cd src
git clone https://github.com/sudrag/beginner_tutorials.git
cd ..
catkin_make
```

## To run

Running requires multiple terminals to be used,

* In Terminal 1 run:

```
roscore
```

Do not close this terminal 

* In Terminal 2 run:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
The output should be similar to: 

`[ INFO] [1509498746.921039616]: Modified String Inserted :104`
`[ INFO] [1509498747.021040830]: Modified String Inserted :105`
`...`

* In Terminal 3 run:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
The output messages should be similar to:

`[ INFO] [1509498746.921492670]: I heard: [Modified String Inserted :104]`
`[ INFO] [1509498747.021447273]: I heard: [Modified String Inserted :105]`
`...`

## Dependencies

* ROS Kinetic
* Catkin
* roscpp package
* std_msgs package
* message_generation package
