# ROS2 Basics

Welcome! This document introduces ROS 2 fundamentals including nodes, topics, services, parameters, and more—demonstrated through C++ examples.
I have covered more C++ side of ros2.

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

ROS 2 is the next generation of ROS, re-engineered to meet the demands of industrial, commercial, and real-time robotics applications. It is built with modern middleware (DDS) and better support for performance, scalability, and security.

ROS 2 was designed to:

   - Fix architectural limitations of ROS 1

   - Support real-time systems

   - Be more scalable and robust

   - Work across multiple platforms (including Windows and embedded systems)

## Bascis of ROS2






