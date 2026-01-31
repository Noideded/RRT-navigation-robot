from controller import Robot, Motor, GPS, Compass
import math
import random

robot = Robot() 
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setPosition(float("inf"))
right_motor.setVelocity(0.0)

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(timestep)
compass.enable(timestep)

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

goal_x, goal_y = 2.11, 0.37

def normalize_angle(a):
    return(a + math.pi) % (2 * math.pi) - math.pi 

def get_robot_pose():
    x, _, y = gps.getValues()
    north = compass.getValues()
    theta = math.atan2(north[0], north[2])
    return x, y, theta 

def collides(x, y):
    for (ox, oy, w, h) in obstacles:
        if (ox - w/2 <= x <= ox + w/2) and (oy - h/2 <= y <= oy + h/2):
            return True 
    return False 

def collides_edge(x1, y1, x2, y2, steps = 20):
    for i in range(steps + 1):
        t = i / steps 
        px = x1 + t * (x2 - x1)
        py = y1 + t * (y2 - y1)
        if collides(px, py):
            return True 
    return False 

class Node:
    def __init__(self, x, y):
        self.x = x 
        self.y = y
        self.parent = None 
        self.cost = 0.0 

def dist(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def find_nearest(tree, x, y):
    return min(tree, key=lambda n: dist(n.x, n.y, x, y))

def steer(from_node, to_x, to_y, step=0.3):
    dx = to_x - from_node.x 
    dy = to_y - from_node.y
    d = math.hypot(dx, dy)

    if d <= step:
        return Node(to_x, to_y)
    
    scale = step/d
    return Node(from_node.x + dx * scale, from_node.y + dy * scale)

def sample_free(goal_x, goal_y, bias=0.1):
    if random.random() < bias:
        return(goal_x, goal_y)
    
    while True:
        x = random.uniform(world_min_x, world_max_x)
        y = random.uniform(world_min_y, world_max_y)
        if not collides(x, y):
            return (x, y)

class NavigationFSM:
    def __init__(self):
        self.state = "FOLLOW"
        self.stuck_threshold = 3.0 
        self.movement_threshold = 0.08 

        self.last_pos = None 
        self.stuck_start_time = None
        self.last_pos_time = None

        self.recovery_attempts = 0 
        self.recovery_action = None
        self.recovery_start_time = None 
        self.recovery_duration = 0 

    def update(self, current_pos, current_time):
        x, y = current_pos 

        if self.last_pos is None:
            self.last_pos = (x, y)
            self.last_pos_time = current_time
            return self.state 
        
        dx = x - self.last_pos[0] 
        dy = y - self.last_pos[1]
        movement = math.sqrt(dx*dx + dy*dy)

        if self.state == "FOLLOW":
            if movement < self.movement_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time 
                else:
                    stuck_duration = current_time - self.stuck_start_time 
                    if stuck_duration > self.stuck_threshold:
                        self.state = "STUCK"
                        self.select_recovery_strategy()
                        print(f"[FSM] Robot is stuck")
            else:
                self.stuck_start_time = None
        elif self.state == "RECOVER":
            if self.recovery_start_time is not None:
                elapsed = current_time - self.recovery_start_time
                if elapsed > self.recovery_duration:
                    if movement > 0.05:
                        self.state = "FOLLOW"
                        self.recovery_attempts = 0 
                        self.stuck_start_time = None
                        print(f"[FSM] Recovery successful")
                    else:
                        self.recovery_attempts += 1
                        if self.recovery_attempts >= 3:
                            self.state = "REPLAN"
                            print(f"[FSM] Max recovery attempts reached")
                        else:
                            self.state = "STUCK"
                            self.select_recovery_strategy()
                            print(f"[FSM] Robot is stuck")
        elif self.state == "REPLAN":
            self.state = "FOLLOW"
            self.recovery_attempts = 0 
            self.stuck_start_time = None
            print(f"[FSM] Replanning - skipping waypoint")

        self.last_pos = (x, y)
        self.last_pos_time = current_time 
        return self.state 

    def select_recovery_strategy(self):
        strategies = [
            ("backward", 1.0),
            ("rotate_left", 1.5),
            ("rotate_right", 1.5),
            ("wiggle", 2.0),
        ]
        weights = [0.4, 0.2, 0.2, 0.2]
        self.recovery_action = random.choices(strategies, weights=weights, k=1)[0]
        self.state = "RECOVER"
        self.recovery_duration = self.recovery_action[1]
        print(f"[FSM] Selected recovery: {self.recovery_action[0]}")

    def reset(self):
        self.state = "FOLLOW" 
        self.last_pos = None
        self.stuck_start_time = None
        self.recovery_attempts = 0 
        self.recovery_action = None 
        self.recovery_start_time = None

    @staticmethod
    def skip_wp(current_pos, wp_pos, next_wp_pos):
        dist_to_wp = math.hypot(wp_pos[0] - current_pos[0], wp_pos[1] - current_pos[1])
        dist_to_next = math.hypot(next_wp_pos[0] - current_pos[0], next_wp_pos[1] - current_pos[1])
        return dist_to_next < dist_to_wp

def navigate_to_waypoint(tx, ty):
    x, y, theta = get_robot_pose()
    dx = tx - x
    dy = ty - y
    dist_err = math.hypot(dx, dy)

    if dist_err < 0.25:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        return True 
    
    target_theta = math.atan2(dy, dx)
    angle_err = normalize_angle(target_theta - theta)

    forward = min(2.0, max(0.5, dist_err * 1.5))
    rotation = 0.8 * angle_err
    
    if abs(angle_err) > 1.05:
        forward *= 0.6 
    elif abs(angle_err) > 0.52:
        forward *= 0.8 

    left = forward - rotation 
    right = forward + rotation 

    max_speed = 6.28
    left_motor.setVelocity(max(-max_speed, min(max_speed, left)))
    right_motor.setVelocity(max(-max_speed, min(max_speed, right)))

    return False

def execute_recovery_step(recovery_action, current_time, recovery_start_time):
    if recovery_start_time is None:
        recovery_start_time = current_time

    action_type, duration = recovery_action
    elapsed = current_time - recovery_start_time 

    if action_type == "backward":
        left_motor.setVelocity(-1.2)
        right_motor.setVelocity(-1.2)

    elif action_type == "rotate_left":
        left_motor.setVelocity(-0.8)
        right_motor.setVelocity(0.8)

    elif action_type == "rotate_right":
        left_motor.setVelocity(0.8)
        right_motor.setVelocity(-0.8)

    elif action_type == "wiggle":
        half_period = 0.5 
        cycle = (elapsed % (2 * half_period)) / half_period
        if cycle < 1.0:
            left_motor.setVelocity(0.8)
            right_motor.setVelocity(-0.8)
        else:
            left_motor.setVelocity(-0.8)
            right_motor.setVelocity(0.8)

    return recovery_start_time

        
robot.step(timestep)

sx, sy, _ = get_robot_pose()
goal_x, goal_y = 2.11, 0.37

start = Node(sx, sy)
tree = [start]
goal_node = Node(goal_x, goal_y)
goal_reached = False

print(f"\n{'='*60}")
print(f"START: ({sx:.2f}, {sy:.2f})")
print(f"GOAL:({goal_x:.2f}, {goal_y:.2f})")
print(f"{'=*60'}\n")

print("[PLANNING] Starting RRT* path planning >>>>")

for i in range(1000):
    rx, ry = sample_free(goal_x, goal_y, bias=0.1) 
    nearest = find_nearest(tree, rx, ry)
    new = steer(nearest, rx, ry, step=0.3)

    if collides(new.x, new.y):
        continue 
    if collides_edge(nearest.x, nearest.y, new.x, new.y):
        continue 

    new.parent = nearest 
    new.cost = nearest.cost + dist(nearest.x, nearest.y, new.x, new.y) 
    tree.append(new)  

    if dist(new.x, new.y, goal_x, goal_y) < 0.2:
        if not collides_edge(new.x, new.y, goal_x, goal_y):
            goal_node.parent = new
            goal_node.cost = new.cost + dist(new.x, new.y, goal_x, goal_y)
            tree.append(goal_node)
            goal_reached = True 
            print(f"[PLANNING] Goal reached at iteration {i}")
            break 
        
    if i % 100 == 0:
        print(f"[PLANNING] Iteration {i}, tree size: {len(tree)}")

if not goal_reached:
    goal_node = find_nearest(tree, goal_x, goal_y)
    print(f"[PLANNING] Goal not reached, using closest node")

path = [] 
n = goal_node 
while n is not None:
    path.append((n.x, n.y))
    n = n.parent
path.reverse() 

path = path[1:]

print(f"[PLANNING] Path found with {len(path)} waypoints\n")

fsm = NavigationFSM() 
wp_index = 0 
recovery_start_time = 0 

print(f"{'='*60}")
print(f"NAVIGATION: Following {len(path)} waypoints")
print(f"{'='*60}\n")

while robot.step(timestep) != -1:
    if wp_index >= len(path):
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print(f"\n{'='*60}")
        print(f"SUCCESS! Path completed with {wp_index} waypoints")
        print(f"{'='*60}")
        break
    
    x, y, theta = get_robot_pose()
    current_time = robot.getTime()

    fsm_state = fsm.update((x,y), current_time)

    if fsm_state == "FOLLOW":
        if wp_index + 1 < len(path):
            if fsm.skip_wp((x, y), path[wp_index], path[wp_index + 1]):
                print(f"[NAV] Skipping intermediate waypoint {wp_index + 1}")
                wp_index += 1 
                fsm.reset()
                continue 
            
        tx, ty = path[wp_index]
        reached = navigate_to_waypoint(tx, ty)

        if reached:
            print(f"[NAV] Waypoint {wp_index + 1}/{len(path)} REACHED")
            wp_index += 1
            fsm.reset()
            recovery_start_time = None

        if wp_index < len(path) and current_time - getattr(navigate_to_waypoint, 'last_print', 0) > 1.0:
            dist_to_wp = math.hypot(path[wp_index][0] - x, path[wp_index][1] - y)
            print(f"[FOLLOW] WP {wp_index + 1}/{len(path)} | Dist: {dist_to_wp:.2f}m | Vel: L={left_motor.getVelocity():.1f} R={right_motor.getVelocity():.1f}")
            navigate_to_waypoint.last_print = current_time
    
    elif fsm_state == "STUCK":
        print(f"[STUCK] Attempting recovery: {fsm.recovery_action[0]}")
    
    elif fsm_state == "RECOVER":
        if fsm.recovery_start_time is None:
            fsm.recovery_start_time = current_time
        
        recovery_start_time = execute_recovery_step(
            fsm.recovery_action,
            current_time,
            fsm.recovery_start_time
        )
    
    elif fsm_state == "REPLAN":
        print(f"[REPLAN] Skipping waypoint {wp_index + 1}")
        wp_index += 1
        fsm.reset()
        recovery_start_time = None
