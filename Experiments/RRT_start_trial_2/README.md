SECOND PROTOTYPE 
This version significantly reduces the lines of code compared to the first prototype. Nested loops have been removed and only one timestep is initialized at the start of the main execution.

The fundamentals of the RRT* path planning remain the same, so this README focuses on improvements and issues encountered. 

**IMPROVEMENTS OVER PREVIOUS VERSION** 

- enchanced navigational systems
  The speed control of the e-puck's wheels is improved. Previously, subtraccting the angle error sometimes caused the robot to spin in place
  
  Rotation speed is now clamped, preventing extreme spinning

- FSM logic
  Recovery strategies are more complex, including an improved wiggle strategy to handle tight obstacles
  
  Movement is now tracked over time and the FSM updates the user on the robot's position
  
  The FSM now limits recovery attempts. After 3 failed recovery attempts, the robot skips the waypoint by entering the "REPLAN" state. 

- Logging

  real time logs of the distance, recovery action, FSM, angle, distance and position

**ISSUES FACED** 
Although this code works and the robot moves and tracks a path, the e-puck starts vibrating violently sometimes while trying to go towards a waypoint, the "STUCK" state is also triggered way too often, causing the "RECOVERY" state to be triggered. As a result, the recovery attempts keep exceeding leading to the skipping of alot of waypoints superficially. 

  


