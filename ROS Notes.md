# Robotics Operating System (ROS)

## Table of Contents

- [Workspace](#workspace)
- [Packages](#packages)
- [Publishers and Subscribers](#publishers-and-subscribers)
- [Messages](#messages)
- [Services](#services)
- [Network Configuration](#network_configuration)
- [Robots](#robots)
  * [Turtlesim](#turtlesim)
  * [Turtlebot](#turtlebot)
- [Commands](#commands)
  * [roscore](#roscore)
  * [rosnode](#rosnode)
  * [rostopic](#rostopic)
  * [rosmsg](#rosmsg)
  * [rossrv](#rossrv)
  * [rosrun](#rosrun)
  * [roscd](#roscd)
- [Others](#others)

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

To understand ROS tobics [READ](understand-ros-topics.pdf)

### Publisher




### Subscriber 

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

7. Check if everything is fine

```sh
rossrv list
```

Read More about [rossrv](#rossrv)

### Use Client/Server

<details>

<summary>area_server.py</summary>

```python
#!/usr/bin/env python

from ros_service_assignment.srv import RectangleAeraService
from ros_service_assignment.srv import RectangleAeraServiceRequest
from ros_service_assignment.srv import RectangleAeraServiceResponse

import rospy

def handle_find_area(req):
    print "Returning [%s * %s = %s]"%(req.width, req.height, (req.width * req.height))
    return RectangleAeraServiceResponse(req.width * req.height)

def find_area_server():
    rospy.init_node('find_area_server')
    s = rospy.Service('find_area', RectangleAeraService, handle_find_area)
    print "Ready to find_area."
    rospy.spin()
    
if __name__ == "__main__":
    find_area_server() 
```
</details>

<details>

<summary>area_client.py</summary>

```python
#!/usr/bin/env python

import sys
import rospy
from ros_service_assignment.srv import RectangleAeraService
from ros_service_assignment.srv import RectangleAeraServiceRequest
from ros_service_assignment.srv import RectangleAeraServiceResponse

def find_area_client(x, y):
    rospy.wait_for_service('find_area')
    try:
        find_area = rospy.ServiceProxy('find_area', RectangleAeraService)
        resp1 = find_area(x, y)
        return resp1.area
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s*%s"%(x, y)
    print "%s * %s = %s"%(x, y, find_area_client(x, y))
```

</details>

In two sperate terminals run the following commands (one terminal per command)

```sh
$ rosrun ros_service_assignment area_server.py
Ready to find_area.
```

```sh
$ rosrun ros_service_assignment area_client.py 3 4
Requesting 3*4
3 * 4 = 12.0
```

---

## Network Configuration

1. 

- 192.168.8.111

---

## Robots

### Turtlesim

#### Motion

<details>

<summary>cleaner.py</summary>

```python
#!/usr/bin/env python

# Q1: What is the limit of rospy.Rate?
# Q2: Is it always good to use large numbers for rospy.Rate? 

from turtlesim.msg import Pose
import rospy
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import Empty

x=0
y=0
z=0
yaw=0

x_min = 0.0
y_min = 0.0
x_max = 11.0
y_max = 11.0


def poseCallback(pose_message):
    # To change the value of the parameters and get the global variables, 
    # "global" keyword is used.  
    global x
    global y, z, yaw
    # Get the information. 
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

def move(speed, distance, is_forward):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()

    #get current location from the global variable before entering the loop 
    x0=x
    y0=y
    #z0=z;
    #yaw0=yaw;

     
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    # assign the x coordinate of linear velocity to the speed.
    if is_forward: 
        velocity_message.linear.x = abs(speed)
    else: 
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

        #task 2. create a publisher for the velocity message on the appropriate topic.  
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    while True :
            #task 3. publish the velocity message 
            velocity_publisher.publish(velocity_message)

            loop_rate.sleep()
            
            #rospy.Duration(1.0)


            #measure the distance moved
            distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            print  distance_moved               
            if not (distance_moved < distance):
                rospy.loginfo("reached")
                break
    
    #task 4. publish a velocity message zero to make the robot stop after the distance is reached
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def rotate(angular_speed, relative_angle, is_clockwise):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()

    #get current location from the global variable before entering the loop 
    x0=x
    y0=y
    #z0=z;
    #yaw0=yaw
    

     
    velocity_message.linear.x = 0 
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0

    # assign the x coordinate of linear velocity to the speed.
    if is_clockwise: 
        velocity_message.angular.z = -abs(angular_speed)
    else: 
        velocity_message.linear.x = abs(angular_speed)


    angle_moved = 0.0
    t0 = time.time()
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second), loop_rate.sleep() will wait for 1/10 seconds   


        #task 2. create a publisher for the velocity message on the appropriate topic.  
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    while True :
            #task 3. publish the velocity message 
            velocity_publisher.publish(velocity_message)

            loop_rate.sleep()
            
            #rospy.Duration(1.0)


            #measure the distance moved
            t1 = time.time()
            angle_moved = angular_speed * (t1-t0)
            print  angle_moved               
            if not (angle_moved < relative_angle):
                rospy.loginfo("reached")
                break
    
    #task 4. publish a velocity message zero to make the robot stop after the distance is reached
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def degrees_to_radians(angle_in_degrees):
    return angle_in_degrees *math.pi /180.0

def get_distance(x1, y1, x2, y2):
    return math.sqrt(pow((x1-x2),2)+pow((y1-y2),2))

def set_desired_orientation (desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = True
    else:
        clockwise = False
	rotate(degrees_to_radians(10), abs(relative_angle_radians), clockwise)

def move_to_goal(goal_pose, distance_tolerance):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()

    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second) 
    E = 0.0

    #task 2. create a publisher for the velocity message on the appropriate topic.  
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    while True:

        kp = 1.0
        ki = 0.02

        e = get_distance(x, y, goal_pose.x, goal_pose.y)
        E = E+e

        velocity_message.linear.x = (kp*e)
        velocity_message.linear.y =0
        velocity_message.linear.z =0
		#angular velocity in the z-axis
        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z =4*(math.atan2(goal_pose.y-y, goal_pose.x-x)-yaw)

        velocity_publisher.publish(velocity_message)

        #rospy.spin()
        loop_rate.sleep()

        if not (get_distance(x, y, goal_pose.x, goal_pose.y)>distance_tolerance):
                rospy.loginfo("reached")
                break
    #task 4. publish a velocity message zero to make the robot stop after the distance is reached
    velocity_message.angular.z = 0
    velocity_message.linear.x = 0 
    velocity_publisher.publish(velocity_message)

def grid_clean():
    loop_rate = rospy.Rate(0.5)
    pose = Pose()
    pose.x=1
    pose.y=1
    pose.theta=0
    move_to_goal(pose, 0.01)
    loop_rate.sleep()
    set_desired_orientation(0)
    loop_rate.sleep()
    
    move(2.0, 9.0, True)
    loop_rate.sleep()
    rotate(degrees_to_radians(10), degrees_to_radians(90), False)
    loop_rate.sleep()
    move(2.0, 9.0, True)
    
    rotate(degrees_to_radians(10), degrees_to_radians(90), False)
    loop_rate.sleep()
    move(2.0, 1.0, True)
    rotate(degrees_to_radians(10), degrees_to_radians(90), False)
    loop_rate.sleep()
    move(2.0, 9.0, True)

    rotate(degrees_to_radians(30), degrees_to_radians(90), True)
    loop_rate.sleep()
    move(2.0, 1.0, True)
    rotate(degrees_to_radians(30), degrees_to_radians(90), True)
    loop_rate.sleep()
    move(2.0, 9.0, True)


    distance = get_distance(x, y, x_max, y_max)

def spiral_clean():
    #declare a Twist message to send velocity commands
    velocity_message = Twist()

    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)


    count = 0

    constant_speed = 4 
    vk = 1
    wk = 2 
    rk = 0.5
    loop_rate = rospy.Rate(1)

    while True:
        rk = rk+1.0 
        velocity_message.linear.x = rk 
        velocity_message.linear.y = 0 
        velocity_message.linear.z = 0 

        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = constant_speed

        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        if not (x<10.5) and (y<10.5):
            velocity_message.linear.x = 0
            velocity_publisher.publish(velocity_message)    



if __name__=='__main__':
    try:
        rospy.init_node('cleaner', anonymous=True)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        option = input("For forward move: 0 \nFor angular move: 1\nFor goal move: 2\nFor grid clean: 3\nFor spiral clean: 4\n")
        
        if option == 0: 
            speed = input("Speed:")
            distance = input("Distance:")
            is_forward = input("Is Forward:")
            move(speed, distance, is_forward)
        elif option == 1: 
            angular_speed = input("Angular Speed:")
            relative_angle = input("Relative Angle:")
            is_clockwise = input("Is Clockwise:")
            rotate(degrees_to_radians(angular_speed), degrees_to_radians(relative_angle), is_clockwise)
        elif option == 2: 
            pose = Pose()
            pose.x=1
            pose.y=1
            pose.theta=0
            move_to_goal(pose, 0.01)
        elif option == 3:
            grid_clean()
        elif option == 4:
            spiral_clean()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.") 
```

</details>

---

## Commands

### roscore

> roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS [nodes](http://wiki.ros.org/Nodes) to communicate. It is launched using the roscore command.

Usage

```sh
roscore
```

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

List all the services

```sh
rossrv list
```


### rosrun

> rosrun allows you to run an executable in an arbitrary package from anywhere without having to give its full path or cd/roscd there first.

Usage:

```sh
rosrun <package> <executable>
```

Example:

```sh
rosrun roscpp_tutorials talker.py
```

### roscd

> roscd allows you to change directories using a package name, stack name, or special location.

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

## Others

### Create shortcuts and aliases in .bashrc (in HOME directory)

Add the following line to make "gb" shortcut to open "Gedit" to edit .bashrc

```sh
alias gb=“gedit /home/riotu/.bashrc” 
```


---