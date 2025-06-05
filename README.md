# ROS2 Basics

Welcome! This document introduces ROS2 fundamentals including nodes, topics, services, parameters, and more demonstrated through C++ examples. I have covered more C++ side of ros2.

## Table of Contents

1. [What is ROS2 ?](#what-is-ros-2)
2. [Basics of ROS2](#basics-of-ros-2)  
   - [Nodes](#nodes)  
   - [Topics](#topics)  
   - [Services](#services)  
   - [Parameters](#parameters)  
3. [Let's Start](#lets-start)  
   - [Create a Colcon Workspace](#create-a-colcon-workspace)  
   - [Create a C++ Publisher](#create-a-c-publisher)  
   - [Create a C++ Subscriber](#create-a-c-subscriber)  
   - [Launch Files](#launch-files)  
4. [Parameters](#parameters-1)  
   - [Create a Parameter](#create-a-parameter)  
5. [Services](#services-1)  
   - [Service Server](#service-server)  
   - [Service Client](#service-client)

## What is ROS ?

ROS (Robot Operating System) is an open-source robotics middleware (not an OS) that provides tools, libraries, and conventions to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Features 

   - Message Passing between processes (nodes) using topics and services.

   - Hardware Abstraction to work with sensors and actuators.

   - Modular Design through reusable packages.

   - Developer Tools like rviz for visualization and rosbag for recording/playback.

## What is ROS2 ?

ROS2 is the next generation of ROS, re-engineered to meet the demands of industrial, commercial, and real-time robotics applications. It is built with modern middleware (DDS) and better support for performance, scalability, and security.

### ROS2 was designed to:

   - Fix architectural limitations of ROS 1

   - Support real-time systems

   - Be more scalable and robust

   - Work across multiple platforms (including Windows and embedded systems)

## Bascis of ROS2

### Nodes

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

![Nodes](/images/Nodes.gif)

### How to run a node ?

```bash
ros2 run <pkg_name> <exec_name>

ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_cpp listener

ros2 run turtlesim turtlesim_node
```

### Some node commands 

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
	
### Some topic commands

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

### Some service commands

```bash
ros2 service list   # gives the list of available services
	
ros2 service list -t   # gives the list of available services along with their types

ros2 service type <service_name>   # prints the type of service

ros2 service info <service_name>   # gives the detailed info about the service

ros2 interface show <service_type>   # gives what structure of msg does the service accepts

ros2 service call <service_name> <service_type> <arguments>   # it calls the service and arguments is optional here

ros2 service find <service_type>   # gives the list of services of same type
```









