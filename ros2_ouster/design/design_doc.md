# ROS2 Ouster LIDAR Driver Design Document

## People

This driver is written by Steve Macenski on his free time and resources. Other contributors or collaborators are extremely welcome to get involved, contribute, and take on maintaince responsibilities.

## Overview

ROS(1) contained drivers for the Ouster OS-1 series of lidars from Ouster. However, these drivers are deeply flawed as will be discussed in the *Existing Solution* section. For ROS2, there are a number of ported capabilities that will improve the code quality and efficiency of the drivers, in addition to new ROS2 features that will further increase reliability and determinism. 

The goal of these drivers is to create a safety certifiable ROS2 Ouster lidar driver utilizing the best in ROS2 features that is design oriented with an eye for future additions.

These drivers will make use of ROS2 components such as:
- Lifecycle nodes
- Component nodes
- Logical Sensor Quality of Service (QoS) settings

In addition to ROS1 capabilities such as:
- TF2
- Services to interact with sensors asynchronously

## Context

In ROS2, there are no existing drivers for the Ouster OS-1 lidar. This lidar is particularly interesting in my opinion and deserves first class support in ROS2. Because of the sensor's focus in autonomous driving, a high bar for software is required. This is what is motivating the design oriented approach.

The hope is that this will become the defacto-standard for the Ouster lidar drivers as ROS2 continues to become widely adopted over ROS1. Additionally, it will act as a good example of a sensor driver with proper abstraction of lidar libraries, data interfaces, and allocation at initialization.

## Goals and Non-Goals

The goal of these drivers is as follows:
- Create a high quality ROS2 driver for the OS-1 lidar
- Create lifecycle and component interfaces to enable new ROS2 features
- Better handle different types of data in the driver to enable future sensors and data through interfaces.
- Designed for future OS lidar series and expansion of processor capabilities

Non-goals are as follows:
- Support additional sensors from other vendors
- Support the OS-2 lidar, at this time
- Safety certify the driver
- Integrate application level, filtering, or higher level functionality in other component nodes.
- Support for non-Linux OS

## Milestones

### Milestone 0
- Achieve ROS1 driver functionality with OS-1 lidar
- Analyze ROS1 driver for positive and negative attributes
- Port relavent code from ROS1 to ROS2 for this driver
- Analyze forks of ROS1 driver for future needs of users

### Milestone 1
- Create functioning basic ROS2 driver (reads packets, gives out image and pointcloud)
- This driver includes lifecycle interfaces but manually called in constructor or on initialization
- Preallocate dynamic memory allocations in the node constructor or configure lifecycle step

### Milestone 2
- This driver includes a component node registration
- This driver is includes a lifeycle node launch file that transitions to active state on bringup
- Create interfaces for OS-1 (or future) sensors and data
- Include unit and integration tests

### Milestone 3
- Publish this driver to ROS Discourse for Autoware and other autonomous driving stakeholders for review
- Resolve stakeholder comments for a V1.0 release
- Test and profile resource utilization

## Existing Solution

There is a ROS1 driver on the Ouster GitHub page with a lightly modified version on [my GitHub](https://github.com/stevemacenski/ouster_ros1.git) page. This driver does not make use of interfaces in the data or sensor to allow for future changes and capabilities. Further, it does not make use of ROS1 nodelets, or a method of intraprocess communication, and will result in substantial overhead in processing packets to clouds, images, and visualization. 

Each of the sensor processing methods are independent nodes requiring serialization between stages. Futher, there is no CI or tests to ensure code is functioning properly. Finally, there is no lifecycle management of the node, which is a new feature in ROS2.

## Proposed Solution

Our proposed solution in ROS2 will take advantage of all the new resources in ROS2. In addition, it will include interfaces for sensors and data to later extend should substantial changes in the sensor be made, support new models, process new data differently, etc.  

### Lifeycle

ROS2 provides Lifecycle nodes to allow us to step programs through stages to create a deterministic bringup. We will use a lifecycle node as the base interface of our driver to allow the allocation, configuration, and activation to be accomplishImageed in a predefined sequence within the node and across other nodes.

### Component

ROS2 provides component nodes to allow us to register multiple nodes into a single process. This allows us to move information between multiple nodes without copying the memory *or serializing* which greatly improves efficiency. This enables entire perception pipelines to be created without serializing or copying the data from the sensor driver. This is equivalent to nodelets in ROS1.

### Data Interface and Implementations

When a packet comes in from the sensor, we need to process it for IMU, PointCloud, Image, and potentially more. Therefore, we need an interface for the data to allow extendability over time. This is particularly important for the `PointCloud2` if we wish to support more point types than a `PointXYZ` (ei `PointXYZi`) as well as the `Image` if we would like to make use of compression or other encoding representations. Since a single packet (LIDAR) could be useful to different data processor interfaces (range image, pointcloud, intensity image, etc), I want to support easy expansions of capabilities for a sensor to process data.

We need a few core implementations:
- Image
- IMU
- PointCloud2

The interface should be relatively simple including sensor packet information, recieved timestamp, if its valid, and conversation to its relavent ROS message type for transport.

### Sensor Interface and Implementation

Rather than assuming we're working with the OS-1 lidar, I'd like to create an interface to the lidar that could have implementations of an OS-1. This allows us to support future lidars from Ouster, such as the OS-2, or custom variants of the OS-1. This could also theoretically support new sensors that adhere to a similar processing method, however it is not the intent to support new sensors.

Sensors need a few core capabilities:
- Poll for new information
- Check if new information is valid
- Convert the new information packet into a data interface implementation
- Pass without copy the data to our drivers
- Grab metadata
- Reset

### ROS Interfaces

#### TF2
- `tf2_msgs/TFMessage` static TF between lidar sensor, IMU, and pointcloud frames

#### Data Sources
- `sensor_msgs/Image` range image
- `sensor_msgs/Image` intensity image
- `sensor_msgs/Image` noise image
- `sensor_msgs/PointCloud2` point cloud

#### Services
- `ouster_msgs/GetMetaData` get sensor configurations and information
- `std_srvs/Empty` reset the sensor in case of a failure

## Testability
Tests that I think need to be created:
- tests covering all free methods
- Interface test for a packet to a point (cloud)
- interface test for a packet to an image
- Interface test for calling services

A goal of 80% test coverage.

## Open Questions

Right now, the ROS2 launch system will not allow for both component nodes to be composed and lifecycle nodes to be stepped through its lifecycle to the activation stage automatically. The example launch files may need to be only stepping through the lifecycle stages but in the future a solution needs to be found to compose lifecycle nodes if there's not an external lifecycle manager.

The exact mechanism for creating sensor data interfaces for the lidar is up in the air. It could equally use a factory as well as make use of templating. 

Some forks of the Ouster driver in ROS1 created factories for multiple types of Pointclouds in PCL to include intensity, range, and noise information. In the base implementation, it will include only a `PointXYZ` cloud, but the interfaces need to be designed to support other cloud types for easy implementation.

Would people prefer if the timestamp was assigned when the packet is read over the sensor's timestamp? This may be helpful if they're not using PTP to synchronize their clocks within a few nanoseconds.