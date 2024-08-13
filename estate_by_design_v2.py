'''
This algorithm uses sparse transition whereby it takes slopes and elevations into account for path planning
Also, it avoides the path planning from considering empty space
This is due to reiteration of not considering it as a voxel grid but a potential field
each grid/cell contains a transition costs
'''

from rasterio_ import create
from APF_3D import *
from PerlinNoise import GenerateHeightMap


def nearest_neighbor_tsp(waypoints):
    if waypoints:
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
                    dist = np.linalg.norm(np.array(current_point) - np.array(point))
                    if dist < min_distance:
                        min_distance = dist
                        nearest_point = point

            # Move to the nearest unvisited waypoint
            sorted_waypoints.append(nearest_point)
            visited.add(nearest_point)
            current_point = nearest_point

        return sorted_waypoints


def waypoint_generator(terrain, slope, spacing):
    waypoints = []

    x_min, x_max, y_min, y_max = 0, terrain.shape[0], 0, terrain.shape[1]

    x_grid = np.arange(0, terrain.shape[0]-1, spacing)
    y_grid = np.arange(0, terrain.shape[1]-1, spacing)

    for i, x in enumerate(x_grid):
        for j, y in enumerate(y_grid):
            if (i % 2 == 0 and j % 2 == 0) or (i % 2 != 0 and j % 2 != 0):
                if slope[x, y] < 15:
                    if 0 < x <= x_max and 0 < y <= y_max:  # Check bounds
                        waypoints.append(
                            (x, y)
                        )
    return waypoints


def main():

    path_points = []

    dataset = r'D:\Internship\dataextraction\0.5m_DSM\N35721E139508_N35651E139595_UM_DSM_25.tif'

    # Target Coordinates
    target_coordinates = [35.68476350248405, 139.54965988466046]

    # Getting the bounding box for Area of Interest, 1 Hectare
    TIFF_Image, AOI, AOI_data, AOI_transform = create.convert_utm2meter(
        target_coordinates[0], target_coordinates[1], dataset
    )

    # Getting the raw terrain and slope data
    data_slope, data_dict, data_array = create.extraction(AOI_data, AOI_transform)

    optimal_terrain_dict, optimal_terrain, elevation_data, optimal_slope = create.redistribute_extreme_slopes(
        AOI_transform, AOI_data,
        data_slope, threshold=15,
        kernel_size=9)

    # generator = GenerateHeightMap()
    # elevation_data, grid, optimal_slope = generator.generate()

    tree_location = waypoint_generator(elevation_data, optimal_slope, spacing=5)

    create.plotting(TIFF_Image, AOI, data_slope, optimal_slope, AOI_data, elevation_data, tree_location)

    print(f"Number of Tree Locations: {len(tree_location)}")

    sorted_waypoints = nearest_neighbor_tsp(tree_location)

    for point in range(0, len(sorted_waypoints) - 1):  # Ensures we do not exceed index
        src = sorted_waypoints[point]
        dest = sorted_waypoints[point + 1]

        path = astar(elevation_data, src, dest, elevation_data, optimal_slope)

        if path:
            # Convert path points to lists of x and y coordinates
            path_points.append([(point[1], point[0]) for point in path])

            print(f"{point+1} paths found out of {len(sorted_waypoints) - 1}")
        else:
            print(f"No path found between {src} and {dest}")

    # Combine all paths into a massive list of tuples
    combined_path = [point for path in path_points for point in path]

    # Smooth the combined path using B-spline
    # smoothed_path = bspline_smooth_path(combined_path)

    if combined_path:
        # for i, path_index in enumerate(smoothed_path):
            # print(f"Path {i} :{path_index}")
        visualize_costs_3d(sorted_waypoints, elevation_data, combined_path)
        visualize_costs_2d(sorted_waypoints, elevation_data, combined_path)

    # Save to CSV
    # saveorno = input("DO u want to save to csv: [Y/N]")
    # if saveorno == "Y":
    #     decision = input("Name:")
    #     decision_csv = decision + ".csv"
    #     create.write2csv(data_dict, decision_csv)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()