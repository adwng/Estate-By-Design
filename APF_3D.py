import numpy as np
import matplotlib.pyplot as plt
import heapq
import random
from PerlinNoise import GenerateHeightMap
import open3d as o3d

class Node:
    def __init__(self, position, cost=float('inf'), parent=None):
        self.position = position
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost


def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))


def astar(grid, start, goal, height_array, slope_array):
    start_node = Node(start, cost=heuristic(start, goal))

    open_set = []
    heapq.heappush(open_set, start_node)
    closed_set = set()

    nodes = {start: start_node}

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.position in closed_set:
            continue

        closed_set.add(current_node.position)

        if current_node.position == goal:
            print("Path Found")
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1]

        neighbors = get_neighbors(current_node.position, grid)
        for neighbor in neighbors:
            neighbor_cost = compute_transition_cost(current_node.position, neighbor, height_array, slope_array)

            if neighbor in closed_set or neighbor_cost == float('inf'):
                continue

            tentative_cost = current_node.cost + neighbor_cost

            if neighbor not in nodes or tentative_cost < nodes[neighbor].cost:
                neighbor_node = Node(neighbor, tentative_cost, current_node)
                nodes[neighbor] = neighbor_node
                heapq.heappush(open_set, neighbor_node)

    return None

def get_neighbors(position, grid):
    neighbors = []
    x, y= position
    directions = [(-1, 1), (1, 1), (-1, 0), (0, -1), (0, 1), (1, 0), (-1, -1), (1, -1)]  # 4-directional movement
    for direction in directions:
        nx, ny= x + direction[0], y + direction[1]
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
            neighbors.append((nx, ny))  # Append new position with direction
    return neighbors


def compute_transition_cost(point_a, point_b, height_array, slope_array):
    ax, ay = point_a
    bx, by = point_b

    # Calculate Euclidean distance
    distance = np.linalg.norm(np.array([ax, ay]) - np.array([bx, by]))

    # Get the slope between points
    slope = slope_array[ax, ay]
    elevation = height_array[ax, ay]

    # Define a cost function that combines distance and slope
    slope_factor = 1 + abs(slope)  # Increase cost with slope
    total_cost = distance * (slope_factor + elevation)**2

    return total_cost


def visualize_costs_3d(locations, grid, total_path):
    # Create a point cloud from the grid
    height, width = grid.shape
    points = []
    colors = []
    way_points = []

    # Normalizing grid for color mapping
    normalized_grid = (grid - np.min(grid)) / (np.max(grid) - np.min(grid))

    # Applying a colormap
    cmap = plt.get_cmap('Greens')  # You can choose any colormap that suits your preferences
    for x in range(width):
        for y in range(height):
            z = grid[y, x]
            points.append([x, y, z])
            color = cmap(normalized_grid[y, x])[:3]  # Convert RGBA to RGB
            colors.append(color)

    points = np.array(points)
    colors = np.array(colors)

    # Create an Open3D point cloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=45))

    for i in locations:
        x, y = i
        z = grid[y,x]
        way_points.append([x, y, z])

    waypoints = o3d.geometry.PointCloud()
    waypoints.points = o3d.utility.Vector3dVector(way_points)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd=point_cloud)

    path_points = np.array([[p[1], p[0], grid[(round(p[0])), round((p[1]))]] for p in total_path])
    lines = [[i, i + 1] for i in range(len(path_points) - 1)]
    path_geometry = o3d.geometry.LineSet()
    path_geometry.points = o3d.utility.Vector3dVector(path_points)
    path_geometry.lines = o3d.utility.Vector2iVector(lines)
    path_geometry.paint_uniform_color([1, 0, 1])

    # Prepare the visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window('3D Map and Path', width=800, height=600)
    vis.add_geometry(mesh)  # Add the mesh instead of point cloud
    vis.add_geometry(waypoints)
    if total_path:
        vis.add_geometry(path_geometry)

    # Run the visualizer
    vis.run()
    vis.destroy_window()


def visualize_costs_2d(waypoints, grid, path):
    fig, ax = plt.subplots(figsize=(10, 10))
    im = ax.imshow(grid, cmap='Greens', origin='lower')

    # Scatter plot for waypoints
    waypoints_x, waypoints_y = zip(*waypoints)  # Extract x and y coordinates from list of tuples
    ax.scatter(waypoints_y, waypoints_x, color='purple', marker='x')  # Note the order of indexing

    # Add a color bar
    plt.colorbar(im, ax=ax, label='Elevation')

    # Plot settings
    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')
    ax.set_title('Animated Path on Map')

    path_x, path_y = zip(*path)
    ax.plot(path_x, path_y, color='red', linewidth='1')

    plt.show()


def random_generator(grid):
    rows, cols = grid.shape
    selected_points = random.sample([(x, y) for x in range(rows) for y in range(cols)], 2)

    src_indice = selected_points[0]
    dest_indice = selected_points[1]

    print(f"Selected source point (x, y): {src_indice}")
    print(f"Selected destination point (x, y): {dest_indice}")

    return src_indice, dest_indice


def main():
    generator = GenerateHeightMap()
    height_array, grid, slope_array = generator.generate()

    # Compute all transition costs
    # transition_costs = compute_all_transition_costs(height_array, slope_array)

    start, goal = random_generator(height_array)

    total_path = []

    # Run A* algorithm
    path = astar(height_array, start, goal, height_array, slope_array)

    # Convert path points to lists of x and y coordinates
    path_points = [(point[1], point[0]) for point in path]

    total_path.append(path_points)

    if total_path:
        visualize_costs_2d(height_array, total_path)


if __name__ == "__main__":
    main()
