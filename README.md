# calibration_imu

## Table of Contents

1. [Overview:](#1-overview) An overview of this package.
2. [Installation:](#2-installation) Instructions for installing this package.
3. [Usage:](#3-usage) Instructions for using this package.
4. [Nodes:](#4-nodes) A detailed description of the calibration nodes available for use.

## 1: Overview

This package includes a number of graphical tools for calibrating accelerometers, gyroscopes, and magnetometers typically found within Inertial Measurement Units (IMUs).

**Keywords:** imu accelerometer gyroscope magnetometer calibration

### 1.1: License

The source code for this package is released under an [MIT license](LICENSE).

**Author/Maintainer:** Paul D'Angio, pcdangio (at) gmail.com

This package has been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## 2: Installation

### 2.1: Building From Source

#### Dependencies

- [Qt 5+](https://doc.qt.io/qt-5/) (software framework for graphical user interfaces)
- [Qt 5+ Data Visualization Module](https://doc.qt.io/qt-5/qtdatavisualization-index.html) (Qt module for 3D data visualization)
- [Robot Operating System (ROS)](http://wiki.ros.org) (robotic software architecture)
- [sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext) (ROS extended sensor messages)
- [ifopt](http://wiki.ros.org/ifopt) (ROS package for IPOPT optimization)
- [rosbag](http://wiki.ros.org/rosbag) (ROS package for saving ROS topic data)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```
cd catkin_workspace/src
git clone https://github.com/pcdangio/ros-calibration_imu.git calibration_imu
cd ../
catkin_make
```

## 3: Usage

This package includes several nodes for calbrating components of your IMU. See the [Nodes](#nodes) section for available nodes to run.

```
rosrun calibration_imu <node_name>
```

## 4: Nodes

This package includes the following nodes for calibration:

- [magnetometer](#41-magnetometer)

### 4.1: magnetometer

This node offers calibration tools for 3-dimensional magnetometers commonly found in IMUs. Magnetometers often experience hard-iron and soft-iron distortions from magnets and metal objects that are statically mounted near the magnetometer. These distortions depend on the design/geometry of your system's hardware. This node will calculate a calibration that reduces these effects and gives a more accurate magnetometer reading.

#### Step 1: Run the node.

The node can be run with the following command:

```
rosrun calibration_imu magnetometer
```

Executing this command will start up the magnetometer calibration GUI.

#### Step 2: Collect data.

The calibration routine needs a set of uncalibrated data to process so it can detect hard-iron and soft-iron distortions. The uncalibrated data is collected by rotating your platform/magnetometer in specific patterns while recording it's data.  To collect data:

1. Ensure your platform is in it's operational configuration. Things like batteries, covers, or other ferrous/magnetic components will all have an effect on the calibration, so they should all be mounted and/or connected during the calibration process.
2. Set up the node to subscribe to your platform's ROS topic for magnetometer data. The magnetometer node automatically subscribes to the `imu/magnetometer` topic, which may be [remapped](http://wiki.ros.org/Remapping%20Arguments) when starting the magnetometer node.
3. Click on the "Start" button in the Data Collection box of the GUI. You will see data start populating in the 3D plot, with a large red point indicating the most recently collected point. The rate at which data is collected can be limited by setting the `max_data_rate` parameter (in Hz) in the ROS parameter server.
