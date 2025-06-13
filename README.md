# ROS2 Basics

Welcome to my ROS2 Basics repository !!

- This document introduces ROS2 fundamentals including nodes, topics, services, parameters, and more demonstrated through C++ examples. 
- I have covered more C++ side of ros2.

## Table of Contents

1. [What is ROS2 ?]
2. [Basics of ROS2] 
   - [Nodes]  
   - [Topics] 
   - [Services] 
   - [Parameters] 
   - [Actions]
3. [Let's Start] 
   - [Create a Colcon Workspace] 
   - [Create a C++ Publisher]
   - [Create a C++ Subscriber]  
   - [Launch Files]  
4. [Parameters]  
   - [Create a Parameter]  
5. [Services]  
   - [Create a Service Server]  
   - [Create a Service Client]
6. [Actions]
   - [Create a Action Server]
   - [Create a Action Client]
   
## What is ROS ?

ROS (Robot Operating System) is an open-source robotics middleware (not an OS) that provides tools, libraries, and conventions to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

#### Key Features 

   - Message Passing between processes (nodes) using topics and services.

   - Hardware Abstraction to work with sensors and actuators.

   - Modular Design through reusable packages.

   - Developer Tools like rviz for visualization and rosbag for recording/playback.

## What is ROS2 ?

ROS2 is the next generation of ROS, re-engineered to meet the demands of industrial, commercial, and real-time robotics applications. It is built with modern middleware (DDS) and better support for performance, scalability, and security.

#### ROS2 was designed to:

   - Fix architectural limitations of ROS 

   - Support real-time systems

   - Be more scalable and robust

   - Work across multiple platforms (including Windows and embedded systems)

## Basics of ROS2

### Nodes

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

![Nodes](/images/Nodes.gif)

#### How to run a node ?

```bash
ros2 run <pkg_name> <exec_name>

ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_cpp listener

ros2 run turtlesim turtlesim_node
```

#### Some node commands 

```bash
ros2 node list   # gives the list of nodes active

ros2 node info <node_name>   # gives the detailed info about a node 

ros2 node list -t   # gives the list of nodes active along with their type
```

### Topics

ROS2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

![Single-Node-Topic](/images/Single-Topic.gif)

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

![Multiple-Node-Topic](/images/Multi-Topic.gif)
	
#### Some topic commands

```bash
ros2 topic list   # gives the list of topics

ros2 topic list -t   # gives the list of topics along with their types

ros2 topic echo <topic_name>   # prints the data published on the topic

ros2 topic info <topic_name>   # gives the detailed info about the topic

ros2 interface show <topic_type>   # gives what structure of msg does the topic accepts

ros2 topic pub <topic_name> <topic_type> '<args>'   # publishes data of given type to the given topic 

ros2 topic hz <topic_name>   # prints the frequency of the topic

ros2 topic bw <topic_name>   # prints the bandwidth of the topic

ros2 topic find <topic_type>   # gives the list of topics of same topic type
```

### Services

Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

![Services](/images/Service.gif)

#### Some service commands

```bash
ros2 service list   # gives the list of available services
	
ros2 service list -t   # gives the list of available services along with their types

ros2 service type <service_name>   # prints the type of service

ros2 service info <service_name>   # gives the detailed info about the service

ros2 interface show <service_type>   # gives what structure of msg does the service accepts

ros2 service call <service_name> <service_type> <arguments>   # it calls the service and arguments is optional here

ros2 service find <service_type>   # gives the list of services of same type
```

### Parameters

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS2, each node maintains its own parameters.

#### Some parameter commands 

```bash
ros2 param list   # gives the list of available parameters of active nodes
	
ros2 param get <node_name> <parameter_name>   # gives the current value of the parameter

ros2 param set <node_name> <parameter_name> <value>   # changes the value of the parameter to the given value

ros2 param dump <node_name>   # prints the parameter's values

ros2 param dump <node_name> > <file_name.yaml>   # stores the parameter's value in the file in yaml format

ros2 param load <node_name> <parameter_file>   # loads the parameter from a file

ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>   # loads the parameters file on startup
```

### Actions 

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

Actions use a client-server model, similar to the publisher-subscriber model. An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

![Actions](/images/Action.gif)

#### Some action commands

```bash
ros2 action list   # gives the list of actions
	
ros2 action list -t   # gives the list of actions along with their types

ros2 action type <action_name>   # prints the type of action

ros2 action info <action_name>   # gives the detailed info about the action

ros2 interface show <action_type>   # gives what structure of msg does the action accepts

ros2 action send_goal <action_name> <action_type> <values>   # sends a goal to the action server. <values> needs to be in yaml format

ros2 action send_goal <action_name> <action_type> <values> --feedback   # if we wish to get feedback
```

## Let's Start

### Create a Colcon Workspace

**`colcon`** (collective construction) is the default build tool for ROS2. It replaces `catkin_make` and `catkin_tools` from ROS1. It can build multiple packages in a workspace, and supports mixed-language packages (e.g., C++, Python).

Key features:

- Parallel building

- Clear dependency management

- Isolated builds

- Out-of-source builds

#### What is ament_cmake ?

**`ament_cmake`** is the CMake-based build system used in ROS2. It replaces the old ROS1 `catkin` system and is used for building C++ packages.

#### What is ament_python ?

**`ament_python`** is the Python build system used in ROS2 for creating and installing Python-based packages. It's commonly used for tools, nodes, and scripts written in Python.

### Steps to create your own colcon c++ workspace 

#### 1. Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

#### 2. Source your ros2 jazzy installation

```bash
source /opt/ros/jazzy/setup.bash
```

#### 3. Go to src folder and create our c++ package

```bash
cd src
ros2 pkg create --build-type ament_cmake <pkg_name>   # any name you want to give 
```

#### 4. Write your first c++ node

Edit your pkg_name/src/main.cpp

```cpp
#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node 
{
public:
  MinimalNode() : Node("minimal_node") 
  {
    RCLCPP_INFO(this->get_logger(), "Hello from ROS2 C++ node!");
  }
};

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalNode>());
  rclcpp::shutdown();
  return 0;
}
```

#### 5. Edit CMakeLists.txt

```bash
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(minimal_node src/main.cpp)
ament_target_dependencies(minimal_node rclcpp std_msgs)

install(TARGETS
minimal_node
DESTINATION lib/${PROJECT_NAME}
)
```

#### 6. Build the workspace

```bash
cd ~/ros2_ws
colcon build
```

#### 7. Source your workspace 

Make sure you are in our ros2_ws directory

```bash
source install/setup.bash
```

#### 8. Run your node

```bash
ros2 run my_cpp_pkg minimal_node 
```

### Create a C++ Publisher 

#### Code for C++ Publisher 

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Publisher : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  	size_t count;
  	
  	void pubStr()
  	{
  		auto msg = std_msgs::msg::String();
		msg.data = "hello" + std::to_string(this->count++);
		std::string param = this->get_parameter("role").as_string();
		RCLCPP_INFO(this->get_logger(), "published: '%s' & param: %s", msg.data.c_str(), param.c_str());
		pub->publish(msg);
  	}

public:
	Publisher() : Node("cpp_pub"), count(0)
	{
		this->declare_parameter("role", "unemployed");
		pub = this->create_publisher<std_msgs::msg::String>("/proficiency", 10);
		timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Publisher::pubStr, this));
		RCLCPP_INFO(this->get_logger(), "cpp publisher initiated");
    };
    
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Publisher>());
	rclcpp::shutdown();
	return 0;
}
```

### Create a C++ Subscriber 

#### Code for C++ Subscriber

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node
{
private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

	void sub_callback(const std_msgs::msg::String &msg)
	{
		RCLCPP_INFO(this->get_logger(), "heard: '%s'", msg.data.c_str());
	}

public:
	Subscriber() : Node("cpp_sub")
	{
		sub = this->create_subscription<std_msgs::msg::String>("proficiency", 10, std::bind(&Subscriber::sub_callback, this, std::placeholders::_1));
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Subscriber>());
	rclcpp::shutdown();
	return 0;
}
```

### Launch Files 

Launch files in ROS2 are Python scripts (.py files) used to start multiple nodes, set parameters, and define configurations in one place. They're part of the launch system that replaced the XML-style launch files from ROS1.

Launch files make it easy to:

- Run your robot simulation

- Bring up drivers

- Start RViz or Gazebo

- Set topic remappings

- Configure nodes with parameters

### How to create a launch file

#### 1. Create a launch directory inside your src 

```bash
mkdir pkg_name/src/launch
```

#### 2. Create a launch files inside your launch directory

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   ld = LaunchDescription()

   publisher_node = Node(
      package="pkg_name",
      executable="file_name",
      name="pub_cpp",
   )

   subscriber_node = Node(
      package="pkg_name",
      executable="file_name",
      name="sub_cpp",
   )

   ld.add_action(publisher_node)   # publisher node
   ld.add_action(subscriber_node)   # subscriber node 
   
   return ld
```

#### 3. Build your package 

```bash
colcon build --packages-select pkg_name
```

#### 4. Run your launch file

```bash
ros2 launch my_robot_package my_robot_launch.py
```

