# Particle Filter Localization - Kidnapped Vehicle Project

## Overview

This project implements a **particle filter** algorithm to **localize a vehicle using map landmarks** to achieve higher positional precision than raw GPS.

The algorithm follows the following steps:

1. Initialize the filter's particles by **randomly sampling from a Gaussian distribution** around the initial GPS position and orientation [x, y, theta].

2. After the vehicle moves for timestep dt with a known **control velocity and yaw rate,*** use CTRV motion equations to **predict the position of the particles** and add **additional Gaussian uncertainty** around these predicted particle positions and orientations.

3. For each particle position, **pre-filter map landmarks** that are within sensor range and **match each of the vehicle's measured observations (RADAR/LIDAR range finding) to the nearest map landmark.**

4. Set each particle's weight by the **multivariate Gaussian probability** of how the measured observation [x, y] position matches the associated map landmark [x, y] position for each of the measured observations, multiplying them all together to get the **total particle weight** value.

5. **Resample** the set of particles with the **probability based on the particle weights** to re-center them on the best matches from the landmark observations.

6. Visualize the **best particle with the highest weight** and it's associated observations as the filter's output vehicle position.

## Result

**Error X:** 0.116

**Error Y:** 0.108


![Image of the end of a Sucessful Run](https://github.com/DPontes/Self-Driving-Car-ND_Particle-Filter-Localization/blob/master/images/Capture1.PNG)

## Key Files

| File                         | Description                                                                                                                                            |
|:----------------------------:|:------------------------------------------------------------------------------------------------------------------------------------------------------:|
| /src/main.cpp                | Source code for **main loop** that handles **uWebSockets communication to simulator**                                                                  |
| /src/particle_filter.cpp, .h | Source code for **particle filter algorithm** that localizes vehicle position by sampling particles that best match measured map landmark observations |
| /build/particle_filter       | Output **executable program binary**                                                                                                                   |
| install-mac.sh               | Script for Mac to install uWebSocketIO required to interface with simulator                                                                            |
| install-ubuntu.sh            | Script for Linux to install uWebSocketIO required to interface with simulator                                                                          |

The original Udacity project repository is [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

## How to Build and Run Code

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two scripts (**install-mac.sh** and **install-ubuntu.sh**) that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
