
# Calibration of IMU Sensors & Orientation Estimation

## Overview
This project involves calibrating the Inertial Measurement Unit (IMU) sensors of the Zumo 32U4 robot and estimating the orientation of an object using sensor fusion techniques. The Zumo 32U4 is a versatile robot controlled by an Arduino-compatible ATmega32U4 microcontroller and equipped with various onboard sensors.

## Zumo 32U4 IMU Sensors
The Zumo 32U4 includes the following onboard inertial sensors that can be used to determine its orientation:
- **ST LSM303D:** A chip combining an accelerometer and a magnetometer.
- **ST L3GD20H:** A gyroscope sensor.

Before using these sensors in any application, calibration is necessary to ensure accurate measurements. Calibration involves comparing the sensor readings with a standard reference and making necessary adjustments to align the measurements with the expected values.

## Lab Session 1: IMU Sensor Calibration
In the first lab session, we covered:
- Connection to the Zumo 32U4 robot via a ROS interface.
- Calibration of IMU sensors.

### Local Coordinate Frame of the Zumo IMU Sensor
![Local Coordinate Frame](img1.png)

## Understanding the 9-DOF IMU
A 9-Degree of Freedom (DOF) IMU consists of:
- **3-axis accelerometer:** Measures proper acceleration with respect to a free-falling body.
- **3-axis gyroscope:** Reports the angular velocity of the object along all three body axes.
- **3-axis magnetometer:** Measures the Earth's magnetic field to determine heading direction with respect to North.

## Lab Session 2: Orientation Estimation
In the second lab session, we covered:
- Implementation of a Kalman filter assuming a linear Gaussian system.
- Estimation of the orientation of a given object using an IMU sensor.

### Robot Orientation Visualization in RViz
![Robot Orientation in RViz](img2.png)
![Robot Orientation Visualization](img3.png)

