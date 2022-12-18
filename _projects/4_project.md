---
layout: page
title: IMU-based 6 DOF Robot Arm Controller
description: An IMU-based 6 DOF robot arm controller as part of NASA's ULI project. 
img: assets/img/IMU.jpg
importance: 4
category: research
---

This research project was part of [NASA's ULI project](https://robotics.wisc.edu/projects/uli/) investigating how shared control and shared autonomy in robotics can assist skilled workers in completing complex and often injury-prone tasks. This project was initially meant to only utilize an IMU for state estimation, however, after testing various algorithms, position estimation was deemed as not viable. 

Various filtering algorithms were tested, such as:

- [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
- [Complementary Filter](https://www.sciencedirect.com/topics/computer-science/complementary-filter)
- [Madgwick Filter](https://ieeexplore.ieee.org/document/5975346)

Various IMUs were tested:

- [Nano 33 BLE's LSM9DS1 IMU](https://store.arduino.cc/products/arduino-nano-33-ble)
- [Adafruit's BNO055 9-DOF IMU Sensor](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- [LMPS-B2 9-axis IMU Sensor](https://lp-research.com/9-axis-bluetooth-imu-lpmsb2-series/)


All the algorithms were tested on ROS and MoveIt. The final controller could control a 6 DOF robot arm using an IMU for orientation, a joystick for 2D/planar control, and two push buttons for the third degree of freedom. Code and instructions for how to recreate the controller can be found [here](https://github.com/itsahmedkhalil/SixDOFRoboticController).

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/HandheldController.jpeg" title="Gazebo Mobile Robot" class="img-fluid rounded z-depth-1" %}
    </div>
</div>


