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

This package includes several nodes for calbrating components of your IMU. See the [Nodes](#4-nodes) section for available nodes to run and more specific instructions on how to use them.

```
rosrun calibration_imu <node_name>
```

## 4: Nodes

This package includes the following nodes for calibration:

- [magnetometer](#41-magnetometer)
- [accelerometer](#42-accelerometer)

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

1. **CRITICAL:** Ensure your platform is in it's operational configuration. Things like batteries, covers, or other ferrous/magnetic components will all have an effect on the calibration, so they should all be mounted and/or connected during the calibration process.
2. **CRITICAL:** Set up your platform in an outdoor area free from nearby magnetic interference (e.g. buildings, power lines, metal plates, etc.) 
3. Set up the node to subscribe to your platform's ROS topic for magnetometer data. The magnetometer node automatically subscribes to the `imu/magnetometer` ([sensor_msgs_ext/magnetic_field](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/magnetic_field.msg)) topic, which may be [remapped](http://wiki.ros.org/Remapping%20Arguments) when starting the magnetometer node. **CRITICAL:** Ensure that the magnetometer is transmitting uncalibrated data through the above topic.
4. Click on the "Start" button in the Data Collection box of the GUI. You will see data start populating in the 3D plot, with a large red point indicating the most recently collected point. The rate at which data is collected can be limited by setting the `~/max_data_rate` parameter (in Hz) in the ROS parameter server.
5. Rotate your platform in all directions. You will be able to see a 3D ellipsoid start forming in the plot window (a red point shows the current position). Once you can clearly see the general shape of a bounded 3D ellipsoid, you may stop collection. **NOTE:** It is very important for the data points to fully bound the ellipsoid, while it is *NOT* important to have a lot of points.

#### Step 3: Run calibration.

The calibration routine can now be executed on the collected data. To calibrate:

1. Enter the true magnetic field strength in the supplied text box. You may look up the true field strength using NOAA's [World Magnetic Model](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm) and your current location on earth. **NOTE:** You must enter the *total* field strength, in nT.
2. Start the calibration. It should take anywhere from 1s-30s depending on how many data points you have. Once the calibration is complete, it will appear in the calibration textbox. If you have all plots enabled, you will be able to see the 3D ellipsoid that was fit to your data, the true magnetic field sphere, and a calibrated set of your data points that should align closely with the true field sphere.
3. You may export your calibration by either copying the calibration text box, saving it to JSON format, or saving it to YAML format.

### 4.2: accelerometer

This node offers calibration tools for 3-dimensional accelerometers commonly found in IMUs. Accelerometers typically exhibit distortions in the form of offset and range scale. This node will calculate a calibration that reduces these effects and gives a more accurate accelerometer reading.

**NOTE:** Accelerometer calibrations should be applied to accelerometer data **BEFORE** any frame transformations.

#### Step 1: Run the node.

The node can be run with the following command:

```
rosrun calibration_imu accelerometer
```

Executing this command will start up the accelerometer calibration GUI.

#### Step 2: Collect data.

The calibration routine needs a set of uncalibrated data points to process so it can detect scale and offset distortions. The uncalibrated data is collected by placing your platform/accelerometer in 6 specific orientations and using the "Grab" functions. To collect data:

1. Set up the node to subscribe to your platform's ROS topic for raw **(not transformed)** accelerometer data. The accelerometer node automatically subscribes to the `imu/accelerometer` ([sensor_msgs_ext/acceleration](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/acceleration.msg)) topic, which may be [remapped](http://wiki.ros.org/Remapping%20Arguments) when starting the accelerometer node. **CRITICAL:** Ensure that the accelerometer is transmitting uncalibrated data through the above topic.
2. Click on the "Start Collection" button in the Data Collection box of the GUI. You will see real-time accelerometer data being populated in the bar chart. The data is taken from a moving average of the actual accelerometer readings. You can set the size of the moving average window by setting the `~/sample_size` parameter in the ROS parameter server.
3. Rotate your platform/accelerometer into each of the 6 indicated orientations, and use the "Grab" button to save the data for each orientation. The orientation label will turn green when it has been successfully grabbed. Some important notes:
    1. **CRITICAL:** The platform/accelerometer must be absolutely motionless during a grab. Rest your platform/accelerometer on stable ground, and do not touch it while grabbing. Use something stable as a stand to assist in holding the platform/accelerometer in the desired orientation.
    2. **NOTE:** It is NOT important for the platform/accelerometer to be exactly in the listed orientation. As long as the platform/accelerometer is within +/-45deg of that orientation, the calibration routine will still function properly (it fits an ellipsoid to the data to derive the actual limits for each axis).


#### Step 3: Run calibration.

The calibration routine can now be executed on the collected data. To calibrate:

1. Enter the true gravity acceleration constant in the supplied text box. You may look up the true field strength using NOAA's [World Magnetic Model](https://geodesy.noaa.gov/cgi-bin/grav_pdx.prl) and your current location on earth.
2. Start the calibration. It should take anywhere from 1s-5s depending on your computer performance. Once the calibration is complete, it will appear in the calibration textbox. The graph will now show the "Fit" and "Calibration" data. The fit is the calculated range of uncalibrated accelerometer data based on the collected data. The fit plot shows the offset and scale errors of the uncalibrated accelerometer. The calibration plot shows the offset and range of the accelerometer after the calibration is applied. This can be used to verify the accuracy of the calibration.
3. You may export your calibration by either copying the calibration text box, saving it to JSON format, or saving it to YAML format.
