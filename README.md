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

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-calibration_imu.git calibration_imu
        cd ../
        catkin_make

## 3: Usage

This package includes several nodes for calbrating components of your IMU. See the [Nodes](#nodes) section for available nodes to run.

        rosrun calibration_imu <node_name>

## 4: Nodes

This package includes the following nodes for calibration:

- [magnetometer](#41-magnetometer)

### 4.1: magnetometer

