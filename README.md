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
