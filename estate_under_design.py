from rasterio_ import create
from AstarFinder3D import PathFinder
# import numpy as np


def main():

    total_path = []

    dataset = r'D:\Internship\dataextraction\0.5m_DSM\N35721E139508_N35651E139595_UM_DSM_25.tif'

    # Target Coordinates
    target_coordinates = [35.68476350248405, 139.54965988466046]

    # Getting the bounding box for Area of Interest, 1 Hectare
    TIFF_Image, AOI, AOI_data, AOI_transform = create.convert_utm2meter(
        target_coordinates[0], target_coordinates[1], dataset
    )

    # Getting the raw terrain and slope data
    data_slope, data_dict, data_array = create.extraction(AOI_data, AOI_transform)

    optimal_terrain_dict, optimal_terrain, elevation_data, optimal_slope = create.redistribute_extreme_slopes(AOI_transform, AOI_data,
                                                                                        data_slope, threshold=15,
                                                                                        kernel_size=9)

    tree_location = create.waypoint_generator(elevation_data, optimal_slope, AOI_transform, AOI)

    print(f"Number of Tree Locations: {len(tree_location)}")
    print(tree_location)

    sorted_waypoints = PathFinder.nearest_neighbor_tsp(tree_location)

    # Create an instance
    path_finder = PathFinder(optimal_terrain, AOI)
    for point in range(0, len(sorted_waypoints) - 1):  # Ensures we do not exceed index
        src = sorted_waypoints[point]
        dest = sorted_waypoints[point + 1]

        path = path_finder.astar(src, dest)

        if path:
            # print(f"Path {i} between {src} and {dest}: {path}")
            total_path.append(path)
            print(f"{len(total_path)} paths found out of {len(sorted_waypoints)}")
        else:
            print(f"No path found between {src} and {dest}")

    if total_path:
        for i, path_index in enumerate(total_path):
            print(f"Path {i} :{path_index}")
        path_finder.visualize_3d_map_and_path_o3d(total_path)

    # Save to CSV
    saveorno = input("DO u want to save to csv: [Y/N]")
    if saveorno == "Y":
        decision = input("Name:")
        decision_csv = decision + ".csv"
        create.write2csv(data_dict, decision_csv)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
