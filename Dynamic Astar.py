import heapq
import numpy as np

class Node:
    def __init__(self, state, g_score, f_score, velocity, steering_angle, parent=None):
        self.state = state  # (x, y, theta)
        self.g_score = g_score
        self.f_score = f_score
        self.velocity = velocity
        self.steering_angle = steering_angle
        self.parent = parent

    def __lt__(self, other):
        return self.f_score < other.f_score


def compute_traversability_cost(elevation, slope, velocity, steering_angle, max_slope):
    elevation_weight = 1.0
    slope_weight = 2.0
    dynamic_weight = 1.5
    steering_weight = 0.5

    elevation_cost = elevation_weight * elevation
    slope_cost = slope_weight * max(0, slope - max_slope)
    dynamic_cost = dynamic_weight * velocity
    steering_cost = steering_weight * abs(steering_angle)

    return elevation_cost + slope_cost + dynamic_cost + steering_cost


def state_transit(cur_state, input, dt, L):
    x, y, theta = cur_state
    v, steer = input
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + (v / L) * np.tan(steer) * dt
    return np.array([x_new, y_new, theta_new])


def get_neighbors(cur_state, velocity, steering_angle, time_interval, max_vel, max_steer, L):
    neighbors = []
    for v in np.arange(0, max_vel + 0.1, 0.5 * max_vel):
        for steer in np.arange(-max_steer, max_steer + 0.1, 0.5 * max_steer):
            input = (v, steer)
            pro_state = state_transit(cur_state, input, time_interval, L)
            neighbors.append((pro_state, v, steer))
    return neighbors


def a_star(start, goal, grid, elevation_map, slope_map, max_velocity, max_slope, time_interval, L, oneshot_range,
           lambda_heu):
    open_set = []
    closed_set = set()
    nodes = {}

    start_node = Node(start, 0, lambda_heu * np.linalg.norm(start - goal), 0, 0)
    goal_node = Node(goal, 0, 0, 0, 0)

    heapq.heappush(open_set, start_node)
    nodes[tuple(start)] = start_node

    while open_set:
        current_node = heapq.heappop(open_set)

        if np.linalg.norm(current_node.state[:2] - goal[:2]) < oneshot_range:
            # Reconstruct path
            path = []
            while current_node:
                path.append(current_node.state)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(tuple(current_node.state))

        neighbors = get_neighbors(current_node.state, current_node.velocity, current_node.steering_angle, time_interval,
                                  max_velocity, np.pi / 4, L)

        for pro_state, velocity, steering_angle in neighbors:
            if tuple(pro_state) in closed_set:
                continue

            x, y, _ = pro_state
            elevation = elevation_map[int(x), int(y)]
            slope = slope_map[int(x), int(y)]
            traversability_cost = compute_traversability_cost(elevation, slope, velocity, steering_angle, max_slope)
            neighbor_cost = current_node.g_score + traversability_cost

            if tuple(pro_state) not in nodes or neighbor_cost < nodes[tuple(pro_state)].g_score:
                h_cost = lambda_heu * np.linalg.norm(pro_state - goal)
                neighbor_node = Node(pro_state, neighbor_cost, neighbor_cost + h_cost, velocity, steering_angle,
                                     current_node)
                nodes[tuple(pro_state)] = neighbor_node
                heapq.heappush(open_set, neighbor_node)

    return None  # Path not found


start = np.array([0, 0, 0])
goal = np.array([99, 99, 0])
grid = np.zeros((100, 100))  # Example grid (all traversable)
elevation_map = np.random.rand(100, 100) * 100  # Elevation values
slope_map = np.random.rand(100, 100) * 30  # Slope values
max_velocity = 5
max_slope = 20  # Maximum slope the robot can handle in degrees
time_interval = 1.0
L = 2.0  # Wheelbase
oneshot_range = 5.0
lambda_heu = 1.0

path = a_star(start, goal, grid, elevation_map, slope_map, max_velocity, max_slope, time_interval, L, oneshot_range, lambda_heu)

if path:
    print("Path found:", path)
else:
    print("No path found")
