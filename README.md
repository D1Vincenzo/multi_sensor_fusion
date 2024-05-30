# Multi-Sensor Fusion for Enhanced Navigation
<img src="latex/figs/readme_pics/cover_page.png" align="right" width="300" alt="header pic"/>
Final Year Individual Project  by Xinyang Huang, University College London. This project is designed and tested on Ubuntu 22.04, ROS2 Humble.

## Table of Contents
- [Introduction](#introduction)
- [Project Report](#project-report)
  - [Report in PDF](#report-in-pdf)
  - [Original LaTeX](#original-latex)
- [Code](#code)
  - [depth_image_to_laserscan](#depth_image_to_laserscan)
  - [robot_urdf](#robot)
  - [multi_sensor_fusion](#multi_sensor_fusion)
- [Test Results](#test-results)
- [Future Work](#future-work)

## Introduction
This repository contains all the necessary code, documentation, and additional resources for the "Multi-Sensor Fusion for Enhanced Navigation" project. The project aims to improve robotic navigation capabilities by integrating data from multiple sensors to overcome the limitations of 2D LiDAR systems.

## Project Report
### Report in PDF
Find the comprehensive project report in PDF format [here](latex/main.pdf).

### Original LaTeX
Access the LaTeX source files used to generate the project report [here](latex).

## Code
This section includes all the source code developed for the project, organized into specific modules:
### depthimage_to_laserscan
Code for converting depth images to laser scan data, enhancing obstacle detection in 2D navigation systems. [More info](src/depthimage_to_laserscan/)

### robot
Implementation of the robotic control algorithms including navigation and sensor integration. [More info](src/fusion_bot/)

### multi_sensor_fusion
Core algorithms for the fusion of multiple sensor data aimed at providing accurate real-time localization and mapping. [More info](src/laser_merger/)

## Test Results
Documentation of testing procedures, results, and how they validate the effectiveness of the proposed solutions. 

With 2D LiDAR only:

![](latex/figs/after_merge.png)

With the fusion algorithm:
![](latex/figs/before_merge.png)

With 2D LiDAR only:

![](figs/readme_pics/barricades.png)

![](figs/readme_pics/barricades_before.png)

With the fusion algorithm:

![](figs/readme_pics/barricades_after.png)
## Future Work
Outline of potential future extensions and improvements to the project, based on current outcomes and technological advancements. 

- Test on real robots.
- Use multiple rows in depth image rather than one row.
- Use multiple cameras to achieve 360 degree detection.

# TODO
Add setup guidance for this project

---
For more information or inquiries, please contact [Xinyang Huang](xinyang.huang.21@ucl.ac.uk).
