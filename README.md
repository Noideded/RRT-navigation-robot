# RRT Navgiation Robot (Webots) 

This repository explores sampling-based path planning and navigation
for mobile robots in Webots.

**OVERVIEW**

This project implements a sampling-based path planning and execution system for the e-puck robot in Webots, using a simplified RRT* (Rapidly-exploring Random Tree Star) algorithm combined with a finite-state machine (FSM) for navigation and recovery.

The system is designed to:

Generate a collision-free path from the robot’s current position to a goal

Navigate the e-puck along the planned waypoints

Detect when the robot becomes stuck

Attempt recovery behaviors and replan if necessary

**FEATURES**

RRT* path planning with goal biasing

GPS + compass-based localization

Differential-drive waypoint tracking

Finite-state machine for navigation and recovery

Recovery behaviors (backward, rotation, wiggle)

Debug logging for planning and execution stages

**SYSTEM ARCHITECTURE**

The controller is structured into three major subsystems:

Perception & Localization

Path Planning (RRT*)

Path Execution & Recovery (FSM)

## Experiments

- **RRT\* e-puck trial**  
  Early prototype combining RRT\* path planning with FSM-based navigation
  and recovery logic.  
  See: `Experiments`
