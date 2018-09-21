# Robotics Oprerating System (ROS)

## Table of Contents

- [Workspace](#workspace)
- [Packages](#packages)
- [Publishers and Subscribers](#publishers-and-subscribers)
- [Messages](#messages)
- [Services](#services)
- [Others](#others)
- [Commands](#commands)
  * [rosnode](#rosnode)
  * [rostopic](#rostopic)
  * [rosmsg](#rosmsg)
  * [rossrv](#rossrv)
  * [rosrun](#rosrun)
  * [roscd](#roscd)

---

## Workspace

### Configure Workspace

1. Add the following command into .bashrc file to activate the ROS default workspace

```sh
source /opt/ros/kinetic/setup.bash
```

### Create Workspace

- Reference: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

- catkin workspace will be used to create and store your own ROS packages (project).

- catkin is the name of the build tool used to compile and execute programs in ROS.

To create catkin workspace (catkin_ws) in HOME directory

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

---

## Packages

> ROS package (project) that you will use to develop programs.

### Create Package

1. Go to the "src" folder

```sh
cd ~catkin_ws/src/
```

2. Create your package (specify the dependencies)

```sh
catkin_create_pkg cs460_package std_msgs rospy roscpp
```

3. Go to catkin_ws and compile to generate executable and configuration files for the project

```sh
cd ..
catkin_make
```

### Make The New Package The Default One

Add the following command in .bashrc (in HOME directory)

```sh
source /home/riotu/catkin_ws/devel/setup.bash 
```

replace riotu by your username

---

## Publishers and Subscribers

---

## Messages

- Syntax: package_name/message_type
- Unsigned int only takes positive values.

---

## Services

### Set up Client/Server

1. Create "srv" folder in the package_name (e.g. ros_service_assignment).

2. Add two dependencies in "package.xml"

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

3. Modify or add the following in "CMakeLists.txt"

- Add "message_generation"

```plain
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

- Add "RectangleAeraService.srv" (.srv file)

```plain
## Generate services in the 'srv' folder
add_service_files(
   FILES
   RectangleAeraService.srv
 )
```

- Uncomment the following

```plain
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
```

- Add "message_runtime"

```plain
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_service_assignment
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

```

4. Create ".srv" file in "srv" folder.

5. Add the request and response values in ".srv" file. Example:

```plain
float32 width
float32 height
---
float32 area
```

6. Run in terminal

```sh
cd catkin_ws
catkin_make
```

---

## Others

### Create shortcuts and aliases in .bashrc (in HOME directory)

Add the following line to make "gb" shortcut to open "Gedit" to edit .bashrc

```sh
alias gb=“gedit /home/riotu/.bashrc” 
```


---

## Commands

### rosnode

>rosnode is a command-line tool for displaying debug information about ROS [Nodes](http://wiki.ros.org/Nodes), including publications, subscriptions and connections. It also contains an experimental library for retrieving node information.

List active nodes

```sh
rosnode list
```

Print information about node

```sh
rosnode info /node_name
```

### rostopic

> The rostopic command-line tool displays information about ROS topics. Currently, it can display a list of active topics, the publishers and subscribers of a specific topic, the publishing rate of a topic, the bandwidth of a topic, and messages published to a topic. The display of messages is configurable to output in a plotting-friendly format.

Display messages published to a topic

```sh
rostopic echo /topic_name
```

Print information about topic

```sh
rostopic info /topic_name
```

Display a list of current topics

```sh
rostopic list
```

### rosmsg

> The rosmsg command-line tool displays information about ROS [Message types](http://wiki.ros.org/msg).

Display the fields in a ROS message type. You may omit the package name of the type, in which case rosmsg will search for matching types in all packages

```sh
rosmsg show std_msgs/String
```

or, if you don't know the package name

```sh
rosmsg show Pose
```

### rossrv 

>The rossrv command-line tool displays information about ROS services. It has the exact same usage as rosmsg (see what it offers when it runs without sub-command below):

```sh

```


### rosrun

>rosrun allows you to run an executable in an arbitrary package from anywhere without having to give its full path or cd/roscd there first.

Usage:

```sh
rosrun <package> <executable>
```

Example:

```sh
rosrun roscpp_tutorials talker.py
```

### roscd

>roscd allows you to change directories using a package name, stack name, or special location.

Usage:

```sh
roscd <package-or-stack>[/subdir]
```

Example:

```sh
roscd roscpp
```

Go to the default workspace 

```sh
roscd
```

---