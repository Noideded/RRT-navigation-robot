from controller import Robot, Motor, GPS, Compass, DistanceSensor
import math 
import random 
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
    
print("Controller started")

gps = robot.getDevice("gps")
gps.enable(timestep) 

compass = robot.getDevice("compass")
compass.enable(timestep) 

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor") 
left_motor.setPosition(float('inf')) 
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0) 

world_min_x, world_max_x = (-2.5, 2.5)
world_min_y, world_max_y = (-2.5, 2.5)

obstacles = [
(0, -0.73, 0.5, 0.5), 
(1.55, -1.24, 0.5, 0.5),
(1.55, 1.18, 0.5, 0.5), 
(-1.26, 0, 0.5, 0.5), 
(-1.26, -2.2, 0.5, 0.5),
(-1.74, -1.71, 0.5, 0.5)
]

class Node:
    def __init__(self, x, y):
        self.x = x 
        self.y = y 
        self.parent = None 
        self.cost = 0.0 
        
    def __repr__(self): 
        return f"Node({self.x:.2f}, {self.y:.2f})" 
        
#FINITE STATE MACHINES 

class Recovery:

    def __init__(self):
        self.state = "FOLLOW" 
        self.stuck_counter = 0 
        self.max_stuck_time = 3.0 
        self.stuck_start_time = None 
        self.last_position = None 
        self.last_position_time = None 
        self.recovery_attempts = 0 
        self.max_recovery_attempts = 3 
        
    def update(self, current_pos, distance_to_target, time_elapsed) :
        x, y = current_pos 
         
        if self.last_position is None: 
            self.last_position = (x, y) 
            self.last_position_time = time_elapsed 
            return False
            
        dx = x - self.last_position[0] 
        dy = y - self.last_position[1]
        movement = math.sqrt(dx*dx + dy*dy) 
         
        if self.state == "FOLLOW":
            if movement < 0.2: 
                if self.stuck_start_time is None:
                    self.stuck_start_time = time_elapsed
                else:
                    stuck_duration = time_elapsed - self.stuck_start_time 
                    if stuck_duration > self.max_stuck_time:
                        if random.random() < 0.8:
                            self.state = "STUCK" 
                            self.stuck_counter = 0 
                            print(f"FSM: State changed to STUCK at position ({x:.2f}, {y:.2f})") 
                            return True 
            else:
                self.stuck_start_time = None 
                   
            self.last_position = (x, y) 
            self.last_position_time = time_elapsed 
        elif self.state == "RECOVER":
            if movement > 0.05:
                self.state = "FOLLOW" 
                self.recovery_attempts = 0 
                print("FSM: Recovery successful! Returning to FOLLOW") 
                return True
                   
        return False 
           
    def get_recovery_act(self):
        if self.state == "STUCK":
            strategies = [
                ("backward", 0.3, 1.0),
                ("rotate", 90, 2.0),
                ("rotate", -90, 2.0),
                ("wiggle", 45, 3),
            ]

            weights = [0.4, 0.2, 0.2, 0.2]
            strategy = random.choices(strategies, weights = weights, k = 1)[0]

            self.state = "RECOVER"  
            self.recovery_attempts += 1

            print(f"FSM: Executing recovery strategy: {strategy[0]}")
            return strategy 
        
        elif self.state == "RECOVER" and self.recovery_attempts >= self.max_recovery_attempts:
            self.state = "REPLAN"
            print("FSM: Too many recovery attempts, requesting replan")
            return ("replan", None, None)

        return None 
                            
    def reset(self):
        self.state = "FOLLOW"
        self.stuck_counter = 0 
        self.stuck_start_time = None 
        self.recovery_attempts = 0 
        print("FSM: Reset to intial state")        
           

#COLLISION DETECTION
def collides(x, y):
    for (ox, oy, w, h) in obstacles:
        if (ox - w/2 <= x <= ox + w/2) and (oy - h/2 <= y <= oy +h/2):
            return True 
    return False 
    
def collides_edge(x1, y1, x2, y2, num_checks = 20):
    dx = x2 - x1 
    dy = y2 - y1 
    
    for i in range(num_checks + 1):
        t = i / num_checks
        px = x1 + t * dx 
        py = y1 + t * dy 
        if collides(px, py): 
            return True          
    return False 
    
#RRT ALGORITHM FUNCTIONS    

def dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
def find_nearest(tree, target_x, target_y): 
    nearest = tree[0]
    min_dist = dist(nearest.x, nearest.y, target_x, target_y)

    for node in tree[1:]:
        d = dist(node.x, node.y, target_x, target_y)
        if d < min_dist:
            min_dist = d 
            nearest = node
    return nearest 
   
def steer(from_node, to_x, to_y, max_step = 0.3):
    dx = to_x - from_node.x
    dy = to_y - from_node.y
    distance = math.hypot(dx, dy)
    
    if distance < max_step:
       return Node(to_x, to_y)   

    scale = max_step / distance 
    new_x = from_node.x + dx * scale 
    new_y = from_node.y + dy * scale

    return Node(new_x, new_y)

def sample_free(goal_x, goal_y, goal_bias = 0.1):

    if random.random() < goal_bias:
        return(goal_x, goal_y)

    while True:
        x = random.uniform(world_min_x, world_max_x)
        y = random.uniform(world_min_y, world_max_y) 
        node = Node(x, y) 
        if not collides(x, y):
            return(x, y) 
        
def find_nearby_nodes(tree, node, radius):
    nearby = []
    for n in tree:
        if dist(n.x, n.y, node.x, node.y) < radius:
            nearby.append(n)
    return nearby 

#PATH FOLLOWING FSM 

def get_robot_pose():
    position = gps.getValues()
    orientation = compass.getValues()

    x = position[0]
    y = position[2]

    theta = math.atan2(orientation[0], orientation[2])

    return x, y, theta 

def navi_to_waypoint(target_x, target_y, linear_speed = 3.0, angular_speed = 2.0):
        
    dx = target_x - x
    dy = target_y - y
    distance_error = math.sqrt(dx*dx + dy*dy)

    if distance_error < 0.10:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        return True 
        
    target_theta = math.atan2(dy, dx)

    angle_error = target_theta - theta 
    angle_error = (angle_error + math.pi) % (2*math.pi) - math.pi 

    k_linear = 2.0 
    k_angular = 4.0 

    forward_speed = min(k_linear * distance_error, 4.0)
    rotation = k_angular * angle_error
        
    left_speed = forward_speed - rotation
    right_speed = forward_speed + rotation

    max_speed = 6.28 
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    if int(robot.getTime()) % 1 == 0:
        print(f"x={x:.2f}, y={y:.2f}, theta={theta:.2f}, dist={distance_error:.2f}")

    return False

def execute_recovery_act(action_type, param1, param2):
    if action_type == "backward":
        desired_dist = param1
        max_time = param2
        start_x, start_y, _ = get_robot_pose()
        start_time = robot.getTime()

        while robot.step(timestep) != -1:
            current_time = robot.getTime()
            if current_time - start_time > max_time:
                print("Backward movement timeout")
                break 
            
            left_motor.setVelocity(-3.0)
            right_motor.setVelocity(-3.0)

            current_x, current_y, _ = get_robot_pose()
            distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            if distance_moved >= desired_dist:
                print(f"Moved backwards by {distance_moved:.2f} meters")
                break

        left_motor.setVelocity(0)
        right_motor.setVelocity(0)

    elif action_type == "rotate":
        target_angle = math.radians(param1)

        start_time = robot.getTime()
        _, _, start_theta = get_robot_pose()
        target_theta = start_theta + target_angle

        while robot.step(timestep) != -1:
            current_time = robot.getTime()
            if current_time - start_time > param2:
                break 
            
            _, _, current_theta = get_robot_pose()
            angle_error = target_theta - current_theta 
            angle_error = (angle_error + math.pi) % (2*math.pi) - math.pi 

            if abs(angle_error) < 0.05:
                break 
            
            rotation_speed = 2.0 * angle_error 
            left_motor.setVelocity(-rotation_speed)
            right_motor.setVelocity(rotation_speed)

        left_motor.setVelocity(0)
        right_motor.setVelocity(0)

    elif action_type == "wiggle":

        wiggle_milli = int(param1 * 1000)
        wiggle_count = param2 

        for _ in range(wiggle_count):
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(-2.0)
            robot.step(wiggle_milli)

            left_motor.setVelocity(-2.0)
            right_motor.setVelocity(2.0)
            robot.step(wiggle_milli)

            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            robot.step(wiggle_milli)

def follow_path_recover(path):
    if not path:
        print("Error: empty path")
        return False
    
    fsm = Recovery()
    waypoint_index = 0 

    print(f"Starting path following with {len(path)} waypoints")

    while waypoint_index < len(path) and robot.step(timestep) != -1:
        current_waypoint = path[waypoint_index]

        x, y, _ = get_robot_pose()
        current_time = robot.getTime()

        fsm.update((x, y), dist(x, y, current_waypoint.x, current_waypoint.y), current_time)
        
        if fsm.state == "FOLLOW":
            success = navi_to_waypoint(current_waypoint.x, current_waypoint.y)

            if success:
                print(f"Reached waypoint {waypoint_index+1}/{len(path)}: {current_waypoint}")
                waypoint_index += 1 

                if waypoint_index < len(path):
                    fsm.reset()
            
            else:
                print(f"Failed to reach waypoint {waypoint_index+1}")

        elif fsm.state == "STUCK":
            print("Robot is stuck, initiating recoevery...")
            recovery_action = fsm.get_recovery_act()
            if recovery_action:
                execute_recovery_act(*recovery_action)

        elif fsm.state == "REPLAN":
            print("Replanning needed...")
            waypoint_index += 1 
            fsm.reset()

            if waypoint_index < len(path):
                print(f"Skipping to waypoint {waypoint_index+1}")
            else:
                print("No more waypoints to try")
                return False
        if waypoint_index >= len(path):
            print("Path completed successfully!")
            return True 
        else:
            print("Path following terminated early")
            return False

#MAIN EXECUTION 

def main():
    robot.step(timestep)

    start_pos = gps.getValues()
    start_x = start_pos[0]
    start_y = start_pos[2] 

    print(f"Starting position: ({start_x:.2f}, {start_y:.2f})")

    goal_x, goal_y = 2.11, 0.37
    print(f"End goal position ({goal_x:.2f}, {goal_y:.2f})")

    start_node = Node(start_x, start_y)

    if collides(start_x, start_y):
        print("Error, starting position is inside an obstacle")
        return 
    
    if collides(goal_x, goal_y):
        print("Error, goal position is inside an obstacle")
        return
    
    #RRT* PLANNING PHASE 
    print("Starting RRT* path planning")

    tree = [start_node]

    max_iterations = 1000
    step_size = 0.3
    neighbour_radius = 1.0 
    goal_tolerance = 0.2 

    goal_node = Node(goal_x, goal_y)
    goal_reached = False

    for iteration in range(max_iterations):
        rand_x, rand_y = sample_free(goal_x, goal_y, goal_bias = 0.1)
        nearest_node = find_nearest(tree, rand_x, rand_y)
        new_node = steer(nearest_node, rand_x, rand_y, step_size)

        if collides(new_node.x, new_node.y):
            continue 
        
        if collides_edge(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
            continue 
        
        nearby_nodes = find_nearby_nodes(tree, new_node, neighbour_radius)

        best_parent = nearest_node
        best_cost = nearest_node.cost + dist(nearest_node.x, nearest_node.y, new_node.x, new_node.y)

        for node in nearby_nodes:
            if not collides_edge(node.x, node.y, new_node.x, new_node.y):
                cost = node.cost + dist(node.x, node.y, new_node.x, new_node.y)
                if cost < best_cost:
                    best_parent = node
                    best_cost = cost

        new_node.parent = best_parent
        new_node.cost = best_cost

        tree.append(new_node)

        for node in nearby_nodes:
            if node == best_parent:
                continue 
            
            new_cost = new_node.cost + dist(new_node.x, new_node.y, node.x, node.y)
            if new_cost < node.cost:
                if not collides_edge(new_node.x, new_node.y, node.x, node.y):
                    node.parent = new_node
                    node.cost = new_cost

        if dist(new_node.x, new_node.y, goal_x, goal_y) < goal_tolerance:
            if not collides_edge(new_node.x, new_node.y, goal_x, goal_y):
                goal_node.parent = new_node 
                goal_node.cost = new_node.cost + dist(new_node.x, new_node.y, goal_x, goal_y) 
                tree.append(goal_node)
                goal_reached = True
                print(f"Goal reached at iteration {iteration}")
                break 
        if iteration % 100 == 0:
            print(f"Iteration {iteration}, tree size {len(tree)}")

    if not goal_reached:
        closest_to_goal = find_nearest(tree, goal_x, goal_y)
        print(f"Goal not reached, closest node is {closest_to_goal}")
        goal_node = closest_to_goal

    path = []
    current_node = goal_node
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent
    path.reverse()

    print(f"Path found with {len(path)} waypoints")
    print("Path waypoints:")
    for i, node in enumerate(path):
        print(f"  {i}: ({node.x:.2f}, {node.y:.2f})")

    if path and len(path) > 1:
        path_follow = path[1:]
        print(f"Following {len(path_follow)} waypoints (skipped current position)")
    else:
        print("Path is too short/empty")
        return 
    
    #PATH EXECUTION WITH FSM 

    print("\n" + "="*50)
    print("Starting path execution with FSM recovery...")
    print("="*50)

    success = follow_path_recover(path_follow)

    if success:
        print("\n Mission accomplished!")
    else:
        print("\n Path execution failed")

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

#RUN MAIN FUNCTION 

if __name__ == "__main__":
    main()



