# Path Planning Using Artificial Potential Fields (APF)

This repository contains C++ implementations of the Artificial Potential Field (APF) method for path planning in mobile robotics. The APF method is a common path planning technique that uses virtual forces to guide a robot from a start position to a goal position while avoiding obstacles.

## Overview

Artificial Potential Field (APF) methods rely on:
- **Attractive Forces**: Pull the robot toward the goal.
- **Repulsive Forces**: Push the robot away from obstacles.

These forces are combined to determine the robot's path in dynamic or static environments.

## Repository Structure

The repository contains the following C++ source files:
- `apf_01.cpp`: Basic implementation of the APF method.
- `apf_02.cpp`: Enhanced APF with improved obstacle avoidance.
- `apf_03.cpp`: APF handling dynamic obstacles.
- `apf_04.cpp`: Optimized APF for real-time applications.
- `apf_t_01.cpp`: Template-based APF implementation for different robot models.
- `iapf_01.cpp`: Intelligent APF with adaptive parameters.
- `iapf_02.cpp`: Further enhanced intelligent APF.
- `iapf_03.cpp`: Intelligent APF combined with machine learning techniques.


## Getting Started

### Prerequisites

Ensure you have the following installed:
- A C++ compiler (e.g., `g++`).
- Standard C++ libraries.

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/HemantP02/path_plannig_apf-t_i.git
   cd path_plannig_apf-t_i
