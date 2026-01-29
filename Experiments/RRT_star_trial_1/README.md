This is the first prototype code for this project, I've used fixed square obstacles for now and a simple e-puck robot for my robot of choice. I decided to add finite state machines to minimize the possibilities of the robot getting stuck in a corner

**LOCALIZATION** 
sensors used:
- GPS (provides global (x, y) position
- Compass (used to compute robot direction (0)

**POSE ESTIMATION**

def get_robot_pose():
    position = gps.getValues()
    orientation = compass.getValues()
    x = position[0]
    y = position[2]
    theta = math.atan2(orientation[0], orientation[2])
    return x, y, theta 

the robot pose is expressed in world coordinates and used consistently across planning and navigation 

**ENVIRONMENT REPRESENTATION** 

the planning space is constrained to a fixed square environment

world_min_x, world_max_x = (-2.5, 2.5)
world_min_y, world_max_y = (-2.5, 2.5)

obstacles are modeled as axis-aligned rectangles 

obstacles = [
(0, -0.73, 0.5, 0.5), 
(1.55, -1.24, 0.5, 0.5),
(1.55, 1.18, 0.5, 0.5), 
(-1.26, 0, 0.5, 0.5), 
(-1.26, -2.2, 0.5, 0.5),
(-1.74, -1.71, 0.5, 0.5)
]

collision checks that the goal points are not inside obstacles, for now they will never be but good to implement, and also checks that the way to the next point is not colliding with an obstacle

**PATH PLANNING RRT***

Node structure 

each node in the tree stores (x, y) position, pointer to parent node, and cumulative path cost 

class Node:
    def __init__(self, x, y):
        self.x = x 
        self.y = y 
        self.parent = None 
        self.cost = 0.0 

**SAMPLING STRATEGY** 

planner uses goal-biased random sampling, sometimes the planner will take a random collision free point, otherwise it will go towards the goal directly. The is for the sake of speed and preserving exploration. 

**TREE EXPANSION** 

sample a target point 
find the nearest node in the tree
steer toward the target by fixed step size
reject nodes that collide with obstacles
add valid nodes to the tree 

**PATH EXTRACTION** 

once the goal is reached/nearby the path is extracted by traversing parent pointer backward from the goal node and reversing the result 

**PATH EXECUTION** 

the robot follows waypoints using a proportional controller. 
the linear velocity scales with distance to waypoint 
angular velocity scales with angular error 

**NAVIGATION FSM** 

FOLLOW - normal waypoint tracking
STUCK - robot is not making progress 
RECOVER - tries to recover the robot 
REPLAN - skips waypoint and resumes navigation for another point 

**STUCK DETECTION** 

FSM flags robot as stuck if insufficient progress is detected for a fixed duration 

**RECOVERY BEHAVIORS** 

move backward 
rotate right/left
wiggle in place 

these actions are randomly selected with weights so the robot tries everything 

**EXECUTION LOOP** 

steps the simulation
updates robot pos 
updates FSM state 
executes navigation or recovery logic
advances to the next waypoint when reached 

loop terminates when simulation ends or all waypoints are reached

**ISSUES OF THIS PROTOTYPE** 

at the start of simulation, the e-puck moves forward slowly, vibrating in the process, then stops. When the robot stopped moving it does not start moving after. 

this is because of the nested loops in the code. There are multiple robot.step loops, what happens is that this causes all the functions to fight for control and eventually leaves the motors in a zero-velocity state with no active loop driving them. 

The solution is to only have one robot.step loop in the entire controller. 










