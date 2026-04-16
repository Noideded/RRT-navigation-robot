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

prox_sensor = [] 
for i in range(8):
    ps = robot.getDevice(f"ps{i}")
    ps.enable(timestep)
    prox_sensor.append(ps)

PS_BASELINE = 100
PS_DETECT = 150 
PS_CRITICAL = 200 
PHANTOM_THRESHOLD = 350 

FRONT = [0, 7]
FRONT_SIDE = [1, 6]

OBSTACLES = [
(0, 0, 0.5, 0.5),
(0, -0.73, 0.5, 0.5), 
(-1.18, 0, 0.5, 0.5),
(-0.48, -1.2, 0.5, 0.5),
(1.55, -1.24, 0.5, 0.5),
(1.55, -0.36, 0.5, 0.5),
(-1.26, 1.37, 0.5, 0.5),
(-1.26, -2.2, 0.5, 0.5),
(-1.74, -1.71, 0.5, 0.5)
]

PHANTOM_OBSTACLES = []

GOAL_X, GOAL_Y = 2.12, 0.64
WORLD_MIN_X, WORLD_MAX_X = -2.5, 2.5 
WORLD_MIN_Y, WORLD_MAX_Y = -2.5, 2.5 
RADIUS = 0.25
BOUNDARY_MARGIN = 0.35 
Iters_per_tick = 10

#helpers 
def normalize_angle(a):
    return(a + math.pi) % (2 * math.pi) - math.pi 

def get_pose():
    x, _, y = gps.getValues()
    theta = math.atan2(compass.getValues()[0], compass.getValues()[2])
    return x, y, theta 

def drive(left, right, limit=6.28):
    left_motor.setVelocity(max(-limit, min(limit, left)))
    right_motor.setVelocity(max(-limit, min(limit, right)))

def collides(x, y):
    current_t = robot.getTime()
    active_phantoms = [(cx, cy, w, h) for cx, cy, w, h, exp in PHANTOM_OBSTACLES if exp > current_t]
    if not (WORLD_MIN_X + RADIUS <= x <= WORLD_MAX_X - RADIUS):
        return True 
    if not (WORLD_MIN_Y + RADIUS <= y <= WORLD_MAX_Y - RADIUS):
        return True 
    for cx, cy, w, h in OBSTACLES + active_phantoms:
        if (cx - w/2 - RADIUS <= x <= cx + w/2 + RADIUS) and \
            (cy - h/2 - RADIUS <= y <= cy + h/2 + RADIUS):
            return True 
    return False

def collides_edge(x1, y1, x2, y2, steps = 40):
    for i in range(steps + 1):
        t = i / steps 
        px = x1 + t * (x2 - x1)
        py = y1 + t * (y2 - y1)
        if collides(px, py):
            return True 
    return False 

def phantom_nearby(px, py, radius=0.4):
    for cx, cy, w, h in PHANTOM_OBSTACLES:
        if math.hypot(px-cx, py-cy) < radius:
            return True 
    return False

def read_ps():
    return [ps.getValue() for ps in prox_sensor]

def front_read(ps_vals):
    return max(ps_vals[i] for i in FRONT) 

def all_read(ps_vals):
    return max(ps_vals[i] for i in FRONT + FRONT_SIDE)

MAIN_SENSORS = [0, 1, 6, 7]

def reactive(base_1, base_r, ps_vals, x, y):
    buffer = math.hypot(x - GOAL_X, y - GOAL_Y)
    if buffer > 1.0:
        return base_1, base_r 
    
    active_sensors = FRONT if buffer < 0.8 else MAIN_SENSORS 

    weights = [
        [-1.8, 1.8],
        [-1.4, 1.4],
        [-0.6, 0.6],
        [0.3, -0.3],
        [-0.3, 0.3],
        [0.6, -0.6],
        [1.4, -1.4],
        [1.8, -1.8],
    ]
    span = PS_CRITICAL - PS_BASELINE
    c1, cr = 0.0, 0.0 
    any_active = False 
    for i in active_sensors:
        val = ps_vals[i]
        if val > PS_BASELINE:
            act = max(0.0, min(1.0, (val - PS_BASELINE) / span))
            if act > 0:
                any_active = True 
                c1 += act * weights[i][0] 
                cr += act * weights[i][1]
    if not any_active:
        return base_1, base_r 
    GAIN = 0.5 if buffer <= 1.0 else 1.5 
    return base_1 + GAIN * c1, base_r + GAIN * cr

#RRT* 
class Node:
    __slots__ = ("x", "y", "parent", "cost")
    def __init__(self, x, y):
        self.x, self.y, self.parent, self.cost = x, y, None, 0.0

class RRTSTARincrement:
    def __init__(self, sx, sy, gx, gy, step=0.25, rewire_r=1, bias=0.15):
        self.gx, self.gy = gx, gy
        self.step = step 
        self.rewire_r = rewire_r 
        self.bias = bias 
        
        self.start = Node(sx, sy)
        self.tree = [self.start]
        self.best_goal = None
        self.iters = 0 
        self.done = False

    def step_once(self):
        if self.done:
            return
        if random.random() < self.bias: 
            rx, ry = self.gx, self.gy
        else:
            rx = random.uniform(WORLD_MIN_X, WORLD_MAX_X)
            ry = random.uniform(WORLD_MIN_Y, WORLD_MAX_Y)
            if collides(rx, ry):
                return
        nn = min(self.tree, key=lambda n: math.hypot(n.x-rx, n.y-ry))
        dx, dy = rx - nn.x, ry - nn.y
        d = math.hypot(dx, dy)
        if d == 0:
            return 
        nx = nn.x + dx/d * min(d, self.step)
        ny = nn.y + dy/d * min(d, self.step)

        if collides(nx, ny) or collides_edge(nn.x, nn.y, nx, ny):
            return 
        
        near = [n for n in self.tree
                if math.hypot(n.x-nx, n.y-ny) <= self.rewire_r]
        par = nn 
        c_min = nn.cost + math.hypot(nn.x-nx, nn.y-ny)
        for n in near:
            c = n.cost + math.hypot(n.x-nx, n.y-ny)
            if c < c_min and not collides_edge(n.x, n.y, nx, ny):
                par, c_min = n, c
        new = Node(nx, ny)
        new.parent = par
        new.cost = c_min
        self.tree.append(new)

        for n in near:
            c = new.cost + math.hypot(new.x-n.x, new.y-n.y)
            if c < n.cost and not collides_edge(new.x, new.y, n.x, n.y):
                anc, is_desc = new.parent, False
                while anc:
                    if anc is n:
                        is_desc = True
                        break
                    anc = anc.parent
                if not is_desc:
                    n.parent = new
                    n.cost = c 

        if math.hypot(nx-self.gx, ny-self.gy) < 0.3:
            if not collides_edge(nx, ny, self.gx, self.gy):
                gc = new.cost + math.hypot(nx-self.gx, ny-self.gy)
                if self.best_goal is None or gc < self.best_goal.cost:
                    g = Node(self.gx, self.gy)
                    g.parent = new
                    g.cost = gc
                    self.best_goal = g 

        self.iters += 1 
        if self.iters >= 1000:
            self.done = True 
            print(f"[RRT*] Planning complete ({self.iters} iters, "f"tree size {len(self.tree)})")
            if self.best_goal:
                print(f"[RRT*] Goal connection found at cost {self.best_goal.cost:.2f}")
            else:
                print(f"[RRT*] Warning: no goal connection, using closest node")


    def get_path(self):
        tip = self.best_goal 
        if tip is None:
            tip = min(self.tree, key=lambda n: math.hypot(n.x-self.gx, n.y-self.gy))
        path, n, seen = [], tip, set()
        while n:
            if id(n) in seen:
                break
            seen.add(id(n))
            path.append((n.x, n.y))
            n = n.parent 
        path.reverse()
        rawpath = path[1:] if len(path) > 1 else []
        return self.smoothpath(rawpath)
    
    def has_goal_path(self):
        return self.best_goal is not None 
    
    @staticmethod
    def smoothpath(path, tries=5):
        path = list(path)
        for _ in range (tries):
            if len(path) < 3:
                break 
            i = random.randint(0, len(path) - 3)
            j = random.randint(i + 2, len(path))
            if not collides_edge(path[i][0], path[i][1],
                                 path[j-1][0], path[j-1][1], steps=60):
                path = path[:i+1] + path[j-1:] 
        return path 

_prev_rot = 0.0

def steer_to(tx, ty, is_final, ps_vals):
    global _prev_rot
    
    x, y, theta = get_pose()
    dist = math.hypot(tx-x, ty-y)
    tolerance = 0.08 if is_final else 0.6

    if dist < tolerance:
        drive(0, 0)
        return True 
    
    angle_err = normalize_angle(math.atan2(ty-y, tx-x) - theta)

    if abs(angle_err) < 0.05:
        angle_err = 0.0
 
    raw_rot = 0.4 * angle_err
    rot = (0.4 * raw_rot) + (0.6 * _prev_rot)
    _prev_rot = rot

    if abs(angle_err) > 1.2:
        if is_final and dist < 1.0: 
            drive(-rot, rot)
            return False 
        fwd = 0.3 
        l, r = fwd - rot, fwd + rot 
        l, r = reactive(l, r, ps_vals, x, y)
        drive(l, r)
        return False 

    heading_factor = max(0.15, math.cos(angle_err))
    fwd = min(5.0, max(0.5, dist * 3.0)) * heading_factor
    
    if is_final and dist < 1.0:
        fwd = max(1.0, dist * 3.0) * heading_factor
        rot = 0.75 * angle_err 
        if abs(angle_err) > 0.5:
            fwd *= 0.3 
        elif abs(angle_err) > 0.2:
            fwd *= 0.6

    l, r = reactive(fwd - rot, fwd + rot, ps_vals, x, y)
    if l + r < 0.5 and front_read(ps_vals) > PS_DETECT:
        scale = 0.5 / max(0.01, (l + r))
        l, r = l * scale, r * scale 
    drive(l, r)
    return False

#avoidance
_crit_active = False
_crit_start_pos = (0.0, 0.0)
_crit_start_time = 0.0 
_crit_cooldown = 0.0 

def avoid_obstacle(ps_vals, x, y, theta, t):
     global _crit_active, _crit_start_pos, _crit_start_time, _crit_cooldown

     fr = front_read(ps_vals)

     if _crit_active: 
         drive(-4.0, -4.0)
         moved = math.hypot(x - _crit_start_pos[0], y - _crit_start_pos[1])
         elapsed = t - _crit_start_time
         if (moved > 0.5 and fr < PS_DETECT) or elapsed > 1.5: 
             _crit_active = False 
             _crit_start_time = t + 3.0 
             print(f"[AVOID] Cleared after {moved:.2f}m - replanning")
             return "replan"
         return "critical"

     if fr >= PS_CRITICAL:
         _crit_active = True 
         _crit_start_pos = (x, y)
         _crit_start_time = t
         obs_x = x + 0.18 * math.cos(theta)
         obs_y = y + 0.18 * math.sin(theta)
         at_bound = (obs_x <= WORLD_MIN_X + 0.5 or obs_x >= WORLD_MAX_X - 0.5 
         or obs_y <= WORLD_MIN_Y + 0.5 or obs_y >= WORLD_MAX_Y - 0.5)
         if not at_bound and not phantom_nearby(obs_x, obs_y) and fr > PHANTOM_THRESHOLD:
             PHANTOM_OBSTACLES.append((obs_x, obs_y, 0.25, 0.25, t + 15.0))
             print(f"[MAP] Phantom ({obs_x:.2f}, {obs_y:.2f}) total = {len(PHANTOM_OBSTACLES)}")
             print(f"[AVOID] Critical front = {fr:.0f} - reversing")
         drive(-4.0, -4.0)
         return "critical"
 
     _crit_active = False
     return False 
            
#recover 
recover_phase = "reverse"
recover_start_pos = (0, 0)
spin_start_theta = 0.0
spin_dir = 1

def recover(start_time, current_time, x, y, ps_vals, theta):
    global recover_phase, recover_start_pos, spin_start_theta

    fr = front_read(ps_vals)
    moved = math.hypot(x - recover_start_pos[0], y - recover_start_pos[1])

    if recover_phase == "reverse":
        drive(-3.0, -3.0)
        if moved > 0.15:
            recover_phase = "spin"
            spin_start_theta = theta
            print(f"[RECOVER] Reversing done . spinning direction - {spin_dir}")

    elif recover_phase == "spin":
        drive(-2.5 * spin_dir, 2.5 * spin_dir)
        theta_diff = abs(normalize_angle(theta - spin_start_theta))
        if theta_diff > 0.785 and fr < PS_DETECT:
            recover_phase = "done"

    elif recover_phase == "done":
        return True 
    
    if current_time - start_time > 25.0:
        recover_phase = "done"

    return False 

class StuckDetector:
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.ref_pos = None
        self.ref_time = None

    def check(self, x, y, t):
        if self.ref_pos is None:
            self.ref_pos, self.ref_time = (x, y), t
            return False 
        if math.hypot(x - self.ref_pos[0], y - self.ref_pos[1]) > 0.20:
            self.ref_pos, self.ref_time = (x, y), t
            return False
        return(t - self.ref_time) > 10.0 
    
def replan(x, y, theta):
    for angle_offset in [math.pi, 0, math.pi/2, -math.pi/2, math.pi*3/4, 
                         -math.pi*3/4, math.pi/4, -math.pi/4]:
        for dist in [0.1, 0.2, 0.3, 0.4, 0.5]:
            rx = x + dist * math.cos(theta + angle_offset)
            ry = y + dist * math.sin(theta + angle_offset)
            if not collides(rx, ry):
                return rx, ry
    return None, None 

wp_index = 0 
path = []
recovering = False 
recover_t = 0.0 

last_print = 0.0 
last_replan = 0
last_ps_log = 0.0 

stuck = StuckDetector()
        
robot.step(timestep)
sx, sy, _ = get_pose()

print(f"\n{'='*55}")
print(f"START: ({sx:.2f}, {sy:.2f})")
print(f"GOAL: ({GOAL_X:.2f}, {GOAL_Y:.2f})")
print(f"Robot navigation start")
print(f"{'='*55}\n")

planner = RRTSTARincrement(sx, sy, GOAL_X, GOAL_Y)

if collides(sx, sy):
    print(f"Error: Start ({sx:.2f}, {sy:.2f}) is in collision!")
else:
    print(f"Start position is acceptable")

print("[NAV] Robot will move and plan in background\n")

while robot.step(timestep) != -1:
    x, y, theta = get_pose()
    t = robot.getTime() 
    ps_vals = read_ps() 
            
    if not planner.done:
        for _ in range(Iters_per_tick):
            planner.step_once() 

    if t - last_replan > 2.0:
            new_path = planner.get_path() 
            if new_path and (planner.done or len(new_path) >= 2):
                new_wp = 0 
                for i, (wx, wy) in enumerate(new_path):
                    if math.hypot(wx - x, wy - y) > 0.3:
                        new_wp = i 
                        break
                path = new_path 
                wp_index = min(new_wp, len(path) - 1)
                stuck.reset() 
                last_replan = t
                print(f"[RRT*] Path updated: {len(path)} waypoints")
                
    if not path:
        drive(-1.5, 1.5)
        if t - last_print > 1.0:
            d = math.hypot(GOAL_X - x, GOAL_Y - y)
            print(f"[NAV] Waiting for path... dist={d:.2f}m "
                  f"(iter {planner.iters})")
            last_print = t
        continue

    result = avoid_obstacle(ps_vals, x, y, theta, t)
    
    if result == "replan":
        PHANTOM_OBSTACLES.clear() 
        rx, ry = replan(x, y, theta)
        if rx is None: 
            print("[REPLAN] No clear start found, skipping")
            continue 
        print(f"[RPLAN] From ({rx:.2f}, {ry:.2f})")
        planner = RRTSTARincrement(rx, ry, GOAL_X, GOAL_Y)
        path = []
        wp_index = 0 
        last_replan = t - 2.1
        continue 
    
    if result == "critical":
        continue 

    if wp_index >= len(path):
        drive(0, 0)
        d = math.hypot(GOAL_X-x, GOAL_Y-y)
        print(f"\n{'='*55}")
        print(f"  {'SUCCESS' if d < 0.15 else 'FAILED'} | "
              f"pos=({x:.2f},{y:.2f}) | dist={d:.2f}m")
        print(f"{'='*55}")
        break

    tx, ty   = path[wp_index]
    is_final = (wp_index == len(path) - 1)

    if recovering:
        done = recover(recover_t, t, x, y, ps_vals, theta)
        if done:
            recovering = False
            print(f"[NAV] Recovery done after {t-recover_t}s at ({x:.2f}, {y:.2f})")
            rx, ry = replan(x, y, theta)
            if rx is not None:
                planner = RRTSTARincrement(rx, ry, GOAL_X, GOAL_Y)
                path = []
                wp_index = 0
                last_replan = t - 5.0
                stuck.reset()
            continue

    reached = steer_to(tx, ty, is_final, ps_vals)

    if reached: 
        print(f"[NAV] WP {wp_index+1}/{len(path)} reached")
        wp_index += 1 
        continue 

    if stuck.check(x, y, t):
        print("[NAV] Robot is surrounded on most side, starting recovery")
        recovering = True
        recover_t = t 
        recover_phase = "reverse"
        recover_start_pos = (x, y)
        spin_dir = 1 if random.random() > 0.5 else -1 
        continue 

    if t - last_print > 1.0:
        d  = math.hypot(tx-x, ty-y)
        lv = left_motor.getVelocity()
        rv = right_motor.getVelocity()
        fr = front_read(ps_vals)
        status = "PLANNING" if not planner.done else "DONE"
        print(f"[NAV] WP {wp_index+1}/{len(path)} | dist={d:.2f}m | "
              f"robot = ({x:.2f}, {y:.2f}) | wp = ({tx:.2f}, {ty:.2f}) | "
              f"L={lv:.1f} R={rv:.1f} | plan={status}({planner.iters})")
        last_print = t

    if t - last_ps_log > 2.0:
        print(f"[PS] {[round(v) for v in ps_vals]}")
        last_ps_log = t
