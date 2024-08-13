import numpy as np
import heapq
import random
import open3d as o3d


# class AntColony():
#
#     def __init__(self, coordinates, number_of_ants, iterations, evaporation_rate=0.4, alpha=3, beta=5, q0=1.9):
#         self.coords = coordinates
#         self.num_ants = number_of_ants
#         self.num_iterations = iterations
#         self.evaporation_rate = evaporation_rate
#         self.alpha = alpha
#         self.beta = beta
#         self.q0 = q0
#
#
#     @staticmethod
#     # Define the distance function (Euclidean distance)
#     def distance(point1, point2):
#         return np.linalg.norm(np.array(point1) - np.array(point2))
#
#     @staticmethod
#     # Define the function to calculate total distance of a path
#     def total_distance(path, distances):
#         total = 0
#         for i in range(len(path) - 1):
#             total += distances[path[i]][path[i + 1]]
#         return total
#
#     # Ant Colony Optimization algorithm
#     def ant_colony_optimization(self):
#         num_cities = len(self.coords)
#         distances = np.zeros((num_cities, num_cities))
#         for i in range(num_cities):
#             for j in range(num_cities):
#                 distances[i][j] = self.distance(self.coords[i], self.coords[j])
#
#         pheromones = np.ones((num_cities, num_cities))
#
#         best_path = None
#         best_distance = float('inf')
#
#         for _ in range(self.num_iterations):
#             for ant in range(self.num_ants):
#                 visited = [False] * num_cities
#                 current_city = random.randint(0, num_cities - 1)
#                 path = [current_city]
#                 visited[current_city] = True
#
#                 while len(path) < num_cities:
#                     probabilities = np.zeros(num_cities)
#                     denom = 0
#                     for city in range(num_cities):
#                         if not visited[city]:
#                             denom += ((pheromones[current_city][city] ** self.alpha) *
#                                       ((1 / distances[current_city][city]) ** self.beta))
#
#                     for city in range(num_cities):
#                         if not visited[city]:
#                             probabilities[city] = ((pheromones[current_city][city] ** self.alpha) *
#                                                    ((1 / distances[current_city][city]) ** self.beta)) / denom
#
#                     if random.random() < self.q0:
#                         next_city = np.argmax(probabilities)
#                     else:
#                         next_city = random.choices(range(num_cities), weights=probabilities)[0]
#
#                     path.append(next_city)
#                     visited[next_city] = True
#                     current_city = next_city
#
#                 path.append(path[0])  # Complete the loop
#
#                 path_distance = self.total_distance(path, distances)
#                 if path_distance < best_distance:
#                     best_distance = path_distance
#                     best_path = path
#
#             # Update pheromone levels
#             pheromones *= (1 - self.evaporation_rate)
#             for i in range(num_cities - 1):
#                 pheromones[best_path[i]][best_path[i + 1]] += 1 / best_distance
#             pheromones[best_path[-1]][best_path[0]] += 1 / best_distance
#
#         sorted_coords = [self.coords[i] for i in best_path[:-1]]
#         return sorted_coords, best_distance


class PathFinder:
    def __init__(self, terrain, bounding_box):

        bbox = bounding_box.bounds
        self.x_min, self.y_min, self.x_max, self.y_max = bbox
        self.z_min = (min(terrain[:, 2]))
        self.z_max = (max(terrain[:, 2])) + 2

        self.x_step = 0.25
        self.y_step = 0.25
        self.z_step = 0.25

        self.array = self.create_grid(terrain)

    def astar(self, src, dest):
        start = self.convCoordsToIndex(src)
        goal = self.convCoordsToIndex(dest)
        print(f"start: {start}")
        print(f"goal: {goal}")

        # Check if the source and destination are unblocked
        if not self.is_unblocked(self.array, start) or not self.is_unblocked(self.array, goal):
            print("Source or the destination is blocked")
            return None

        close_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        open_list = []
        heapq.heappush(open_list, (f_score[start], start))

        while open_list:  # while open list still has elements
            current = heapq.heappop(open_list)[1]

            if self.is_destination(current, goal):  # check if current popped index is close to destination
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                    # Updates the current node to its parent node, moving one step backward along the path.
                data.append(start)
                data.reverse()
                return data

            close_set.add(current)  # add current popped index to closed set to marked as visited

            neighbors = [
                (current[0] + dx, current[1] + dy, current[2] + dz)
                for dx, dy, dz in [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)]
                if 0 <= current[0] + dx < self.array.shape[0] and
                   0 <= current[1] + dy < self.array.shape[1] and
                   0 <= current[2] + dz < self.array.shape[2] and
                   current[2] + dz >= 0  # Ensure z is within bounds
            ]

            for neighbor in neighbors:
                if self.array[neighbor] == 1:  # Check if neighbor is an obstacle
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None

    def convCoordsToIndex(self, point):
        x, y, z = point  # Extract x, y, and z coordinates

        print(f"Point: {x},{y},{z}")

        # Check if the point falls within the grid boundaries
        if self.x_min <= x < self.x_max and self.y_min <= y < self.y_max and self.z_min <= z < self.z_max:
            # Convert the continuous coordinates to grid indices
            i = int((x - self.x_min) / self.x_step)
            j = int((y - self.y_min) / self.y_step)
            k = int((z - self.z_min) / self.z_step)

            return (i, j, k)

    def create_grid(self, mapdata):

        # Create a grid within the AOI bounds
        x_grid = np.arange(self.x_min, self.x_max, self.x_step)
        y_grid = np.arange(self.y_min, self.y_max, self.y_step)
        z_grid = np.arange(self.z_min, self.z_max, self.z_step)
        X, Y, Z = np.meshgrid(x_grid, y_grid, z_grid, indexing='ij')

        # Initialize the grid with zeros
        grid = np.zeros_like(X)

        # Iterate through each point in the optimal array
        for point in mapdata:
            x, y, z = point  # Extract x, y, and z coordinates

            # Check if the point falls within the grid boundaries
            if self.x_min <= x < self.x_max and self.y_min <= y < self.y_max and self.z_min <= z < self.z_max:
                # Convert the continuous coordinates to grid indices
                i = int((x - self.x_min) / self.x_step)
                j = int((y - self.y_min) / self.y_step)
                k = int((z - self.z_min) / self.z_step)

                # Mark the corresponding grid cell as 1
                grid[i, j, k] = 1

        return grid

    @staticmethod
    def nearest_neighbor_tsp(waypoints):
        # Create a list to store the sorted waypoints
        sorted_waypoints = []
        # Create a set to keep track of visited waypoints
        visited = set()

        # Start with the first waypoint in the list
        current_point = waypoints[0]
        sorted_waypoints.append(current_point)
        visited.add(current_point)

        # Continue until all waypoints are visited
        while len(visited) < len(waypoints):
            min_distance = float('inf')
            nearest_point = None

            # Find the nearest unvisited waypoint to the current point
            for point in waypoints:
                if point not in visited:
                    x1, y1, z1 = current_point
                    x2, y2, z2 = point
                    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
                    if dist < min_distance:
                        min_distance = dist
                        nearest_point = point

            # Move to the nearest unvisited waypoint
            sorted_waypoints.append(nearest_point)
            visited.add(nearest_point)
            current_point = nearest_point

        print("Waypoints Sorted")

        return sorted_waypoints

    @staticmethod
    def is_destination(cur, dest):
        # return np.sqrt((dest[0] - cur[0]) ** 2 + (dest[1] - cur[1]) ** 2 + (dest[2] - cur[2]) ** 2) <= 1
        return abs(cur[0] - dest[0]) <= 1 and abs(cur[1] - dest[1]) <= 1 and abs(cur[2] - dest[2]) <=1

    @staticmethod
    def is_unblocked(grid, cur):
        i, j, k = cur
        return grid[i][j][k] == 0

    @staticmethod
    def heuristic(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def visualize_3d_map_and_path_o3d(self, paths):
        # Create a point cloud from the map
        points = np.argwhere(self.array > 0)  # Extract coordinates where map elements are non-zero
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        # Estimate normals for the point cloud
        point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # Assign colors based on z-coordinate (height)
        colors = []
        for point in points:
            # Normalize z-coordinate to range [0, 1]
            normalized_z = (point[2] - np.min(points[:, 2])) / (np.max(points[:, 2]) - np.min(points[:, 2]))
            color = [0.5*normalized_z, 0.7*normalized_z, 0.2*normalized_z]
            colors.append(color)

        point_cloud.colors = o3d.utility.Vector3dVector(colors)

        # Convert point cloud to mesh using Poisson surface reconstruction
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd=point_cloud)
        path_geometries = []
        if paths:
            # Create a path line set for each path in the list

            for path_points in paths:
                if path_points:
                    lines = [[i, i + 1] for i in range(len(path_points) - 1)]  # Lines between consecutive points
                    path_geometry = o3d.geometry.LineSet()
                    path_geometry.points = o3d.utility.Vector3dVector(path_points)
                    path_geometry.lines = o3d.utility.Vector2iVector(lines)
                    path_geometry.paint_uniform_color([random.uniform(0, 1),
                                                       random.uniform(0, 1),
                                                       random.uniform(0, 1)]
                                                      )
                    path_geometries.append(path_geometry)

        # Prepare the visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window('3D Map and Path', width=800, height=600)
        vis.add_geometry(mesh)  # Add the mesh instead of point cloud
        if paths:
            for path_geometry in path_geometries:
                vis.add_geometry(path_geometry)

        # Run the visualizer
        vis.run()
        vis.destroy_window()


# def random_generator(grid, x_min, x_step, y_min, y_step, z_min, z_step):
#     # Identifying all unoccupied points in the grid
#     unoccupied_indices = np.where(
#         grid == 0)  # Returns a tuple of arrays, each containing the indices of the 0 elements in the grid
#
#     # Convert indices to list of coordinates
#     unoccupied_coordinates = list(zip(unoccupied_indices[0], unoccupied_indices[1], unoccupied_indices[2]))
#
#     # Randomly select two distinct unoccupied points
#     selected_points = random.sample(unoccupied_coordinates, 2)  # Selects 2 unique random elements
#
#     src_index = (selected_points[0][0], selected_points[0][1], selected_points[0][2])
#     dest_index = (selected_points[1][0], selected_points[1][1], selected_points[1][2])
#
#     # Convert grid indices back to actual coordinates
#     src = (x_min + selected_points[0][0] * x_step, y_min + selected_points[0][1] * y_step,
#            z_min + selected_points[0][2] * z_step)
#     dest = (x_min + selected_points[1][0] * x_step, y_min + selected_points[1][1] * y_step,
#             z_min + selected_points[1][2] * z_step)
#
#     print(f"Selected source point (x, y, z): {src}")
#     print(f"Selected destination point (x, y, z): {dest}")
#
#     print(f"Selected source point (x, y, z): {src_index}")
#     print(f"Selected destination point (x, y, z): {dest_index}")
#
#     return src_index, dest_index


# def main():
#     dataset = r'D:\Internship\dataextraction\0.5m_DSM\N35721E139508_N35651E139595_UM_DSM_25.tif'
#
#     # Target Coordinates
#     target_coordinates = [35.68476350248405, 139.54965988466046]
#
#     # Getting the bounding box for Area of Interest, 1 Hectare
#     TIFF_Image, AOI, AOI_data, AOI_transform = create.convert_utm2meter(
#         target_coordinates[0], target_coordinates[1], dataset
#     )
#
#     # Getting the raw terrain and slope data
#     data_slope, data_dict, data_array, optimal_slope, optimal_dict, optimal_array, tree_location = (create.
#     extraction(
#         AOI, AOI_data, AOI_transform
#     ))
#
#     bbox = AOI.bounds
#     x_min, y_min, x_max, y_max = bbox
#     z_min = round(min(optimal_array[:, 2]))
#
#     # Define grid parameters
#     x_step = 0.25  # grid step along x-axis
#     y_step = 0.25  # grid step along y-axis
#     z_step = 0.25  # grid step along z-axis
#
#     src, dest = random_generator(, x_min, x_step, y_min, y_step, z_min, z_step)
#
#     # Create an instance of PathFinder
#     path_finder = PathFinder(optimal_array, AOI)
#     path = path_finder.astar(src, dest)
#
#     if path:
#         print("Path: ", path)
#         path_finder.visualize_3d_map_and_path_o3d(path)
#     else:
#         print("No path found")
#
#
# if __name__ == '__main__':
#     main()
