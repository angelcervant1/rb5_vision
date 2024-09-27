# Depth Estimation for Monocular Camera With the Qualcomm RB5 Drone

## Overview

This repository contains the implementation of a depth estimation system using a monocular camera mounted on the Qualcomm RB5 Drone. The goal of the project is to leverage computer vision techniques to estimate depth information from a single camera feed, which can be useful for various drone applications such as obstacle avoidance, navigation, and 3D scene reconstruction.
## Example

<table>
  <tr>
    <td align="center">
      <img src="src/depth_estimator/images/imagecopy3.png" alt="Object Detection" width="400"/><br>
      <b>Object Detection</b>
    </td>
    <td align="center">
      <img src="src/depth_estimator/images/imagecopy2.png" alt="Depth Estimation" width="400"/><br>
      <b>Depth Estimation</b>
    </td>
  </tr>
</table>

## Features

- Monocular Depth Estimation: Extracts depth information from a single camera image stream using deep learning-based techniques.
- Integration with Qualcomm RB5 Drone: Utilizes the onboard processing capabilities of the Qualcomm RB5 platform for real-time depth estimation.
- ROS Compatibility: Built within the ROS (Robot Operating System) framework to ensure easy integration with other drone control and navigation systems.
- Real-time Performance: Optimized for real-time depth processing to support dynamic drone maneuvers and applications.


## Requirements

### Hardware

  - Qualcomm RB5 Drone

### Software

  - Docker>=20.0
  - Ubuntu 20.04 (tested on Qualcomm RB5 with ROS Melodic)

### To test out the ROS Node

1. Clone the repository 
2. Build the package with `catkin_make`
3. Build the docker image with `docker build -t rb5_vision .`
4. Make sure to add docker to xhost group with `xhost +local:docker` on a terminal
5. Enter the container with `docker run --gpus all --network=host -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/video0:/dev/video0 --rm rb5_vision bash`
6. If previous code through an nvidia-runtime error. Then ignore the `--gpus all` flag and run the following command: `docker run --network=host -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/video0:/dev/video0 --rm rb5_vision bash`
7. When inside the container run the following command `roslaunch depth_estimator depth_model.launch`

