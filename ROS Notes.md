# Robotics Oprerating System (ROS)

## Table of Contents

- [Workspace](#Create-Workspace)
- [Publishers and Subscribers](#Publishers-and-Subscribers)
- [Messages](#Messages)
- [Services](#Services)
- [Commands](#Commands)

---

## Workspace

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

3. Create ".srv" file in "srv" folder.

4. Add the request and response values in ".srv" file. Example:

```plain
float32 width
float32 height
---
float32 area
```

5. Run in terminal 

```sh
cd catkin_ws
catkin_make
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

## rosmsg

> The rosmsg command-line tool displays information about ROS [Message types](http://wiki.ros.org/msg). The following sections describe the commands that are available

Display the fields in a ROS message type. You may omit the package name of the type, in which case rosmsg will search for matching types in all packages

```sh
rosmsg show std_msgs/String
```

or, if you don't know the package name

```sh
rosmsg show Pose
```

---