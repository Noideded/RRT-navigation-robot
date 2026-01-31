Second prototype of the project, significantly cut down the lines of code for this version of the project. Nested loops from the first prototype are now removed, only one timestep remains and it is initialized during the start of the main chunk of the code. 

The fundamentals stay the same, I won't be explaining the way the RRT* works in this readme, as it is basically identical to the readme of the first prototype. I will be explaining what improvements this one has and also what issues I have encountered now. 

**IMPROVEMENTS OVER PREVIOUS VERSION** 

- enchanced navigational systems
  before the speed control of the e-puck rotational speed for both the left and right wheel was problematic.
  The speed was deduced by subtracting the angle of error, so if the angle error was too large the robot will
  spin in place

  now in the new code the speed of rotation is clamped

- FSM logic

  more complex recovery strategies such as a more complex wiggle strategy, to maneuver complex obstacles. Thecode now also tracks movement over time and updates the user on the position of    the bot. The first prototype also does not trac how long each recovery takes, once the robot gets stuck again during recovery it repeats "RECOVERY" without limit. In the second code after   3 recovery attempts, the bot skips waypoint by going to "REPLAN".

- Logging

  real time logs of the distance, recovery action, FSM, angle, distance and position

**ISSUES FACED** 
Although this code works and the robot moves and tracks a path, the e-puck starts vibrating violently sometimes while trying to go towards a waypoint, the "STUCK" state is also triggered way too often, causing the "RECOVERY" state to be triggered. As a result, the recovery attempts keep exceeding leading to the skipping of alot of waypoints superficially. 

  


