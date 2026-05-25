# Autonomous Mobile Robot

This is an autonomous navigation system developed for the iRobot Create and validated in simulation. The system combines pose estimation, roadmap planning and differential drive control to execute waypoint missions while accounting for uncertainty in sensor and odometry data.

## Overview

This repository is built around a practical robotics pipeline. The main mission driver initializes localization, builds a collision-free path through the environment and executes the plan with a low-level controller. The design keeps perception, planning, and control separated while ensuring filtered state estimates flow cleanly between subsystems.

## Features

- Particle filter localization with odometry and depth measurements
- Optional EKF module for pose estimation and measurement gating
- Probabilistic Roadmap (PRM) planning with collision-checked edges
- Shortest Hamiltonian path ordering for waypoint missions
- Differential drive execution via feedback linearization
- Simulated and real-world deployment on iRobot Create

## Structure

AMR-Final-Project/
├── finalCompetition.m           # Main mission orchestrator
│
├── State_Estimation/            # SLAM, Localization & Sensor Noise Handling
│   ├── EKF.m
│   ├── PF.m
│   ├── initLocalize.m
│   └── depthPredict.m
│
├── Motion_Planning/             # Path generation and optimization
│   ├── buildPRM.m
│   ├── buildCostMatrix.m
│   ├── shortestHamiltonianPath.m
│   ├── extractPath.m
│   └── visitWaypoints.m
│
├── Control/                     # Low-level kinematics
│   ├── feedbackLin.m
│   └── limitCmds.m
│
├── Utils/                       # I/O, logging, and visualization
│   ├── readStoreSensorData.m
│   └── plotAMRMap.m
│
└── Data/                        # Map and waypoint files
    ├── PracticeMap2026.mat
    ├── PracticeMap2026.txt
    ├── practiceMap2026_4credit.mat
    └── practiceMap2026_4credit.txt

## Usage

Run the main pipeline from MATLAB:

```matlab
addpath(genpath(pwd));
Robot = iCreateRobot('COM3');
SetFwdVelAngVelCreate(Robot, 0, 0);
dataStore = finalCompetition(Robot, 420, 0.13, 0.0);
```

## Notes

The current runtime flow uses `PF.m` for online localization, while `EKF.m` is included as a formal estimation module. Planning is built on a collision-aware roadmap and waypoint ordering for efficient mission execution.

## Data

Included map and waypoint files support simulation and testing of the navigation pipeline.
