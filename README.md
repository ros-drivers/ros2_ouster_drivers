# ROS2 Ouster Drivers

These are an implementation of ROS2 drivers for the Ouster OS-1 3D lidars. This includes all models of the OS-1 from 16 to 128 beams. 

TODO
[include a cool gif of working]

TODO
[Links to youtube videos / channel / subscribe]

## Documentation

Documentation can be generated using Doxygen. 

## Design

See design doc in `design/*` directory [here](design/design_doc.md).

## Interfaces

TODO create table
`sensor_msgs/Image` range image topic
`sensor_msgs/Image` intensity image topic
`sensor_msgs/Image` noise image topic
`sensor_msgs/PointCloud2` pointcloud topic

TODO services


## Examples

TODO
[more gifs and point to ROS1 driver / youtube videos again]

## Lifecycle

This ROS2 driver makes use of Lifecycle nodes. If you're not familiar with lifecycle, or managed nodes, please familiarize yourself with the [ROS2 design](https://design.ros2.org/articles/node_lifecycle.html) document on it.

The lifecycle node allow for the driver to have a deterministic setup and tear down and is a new feature in ROS2. The launch script will need to use a lifecycle node launcher to transition into the active state to start processing data.

## Component

This ROS2 driver makes use of Component nodes. If you're not familiar with component nodes please familiarize yourself with the [ROS2 design](https://index.ros.org/doc/ros2/Tutorials/Composition) document on it.

The component node allow for the driver and its processing nodes to be launched into the same process and is a new feature in ROS2. This allows the sensor and its data clients to operate without serialization or copying between nodes sharing a memory pool.

There's a little work in ROS2 Eloquent to launch a component-lifecycle node using only the roslaunch API. It may be necessary to include the Ouster driver in your lifecycle manager to transition into the active state when loading the driver into a process as a component.

# Ouster Messages

A message `Metadata` was created to describe the metadata of the lidars. In addition the `GetMetadata` service type will be used to get the metadata from a running driver.
