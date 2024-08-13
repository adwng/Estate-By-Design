import rasterio
from rasterio.mask import mask
from rasterio.plot import show
from pyproj import Proj
from shapely.geometry import box
import csv
from matplotlib import pyplot as plt
import numpy as np
import richdem as rd


class create:
    @staticmethod
    def convert_utm2meter(target_coordinate_left_lat, target_coordinate_left_lon, data_set):
        data = rasterio.open(data_set)

        # Define the UTM zone
        utm_zone = Proj(proj='utm', zone=54, ellps='WGS84')

        # Convert the bounding box coordinates from UTM to latitude and longitude
        utm_x, utm_y = utm_zone(target_coordinate_left_lon, target_coordinate_left_lat, inverse=False)
        # Calculate the bounding box coordinates
        half_side = 25  # Half the side length of the box (100 meters / 2)
        min_x = utm_x - half_side
        max_x = utm_x + half_side
        min_y = utm_y - half_side
        max_y = utm_y + half_side

        # Create AOI box using Shapely
        aoi_box = box(min_x, min_y, max_x, max_y)

        AOI_data, AOI_transform = rasterio.mask.mask(data, [aoi_box], crop=True, all_touched=True)

        return data, aoi_box, AOI_data, AOI_transform

    @staticmethod
    def extraction(aoi_data, aoi_transform):
        dictionary = []

        # Find the raw AOI's slope
        terrain = rd.rdarray(aoi_data[0], no_data=np.nan)
        slope = rd.TerrainAttribute(terrain, attrib='slope_degrees')

        # Extract x, y, and elevation data, and then append all with slope to a dictinary

        for i in range(0, aoi_data.shape[1]):
            for j in range(0, aoi_data.shape[2]):
                # Calculate the coordinates of the current pixel
                lon, lat = rasterio.transform.xy(aoi_transform, i, j)

                point = {
                        "Longitude": lon,
                        "Latitude": lat,
                        "elevation": aoi_data[0, i, j],
                        "slope": slope[i, j]
                    }
                # Create a dictionary for the current point
                dictionary.append(point)

        raw_data_list = np.array(
            [[key['Longitude'], key['Latitude'], key['elevation']] for key in dictionary]
        )

        return slope, dictionary, raw_data_list

    @staticmethod
    def waypoint_generator(terrain, slope, aoi_transform, AOI):
        waypoints = []

        x_min, y_min, x_max, y_max = AOI.bounds

        # Set a grid space of 9 meters
        x_grid = np.linspace(0, terrain.shape[0] - 9, 1, dtype=int)
        y_grid = np.linspace(0, terrain.shape[1] - 9, 1, dtype=int)

        for i, x in enumerate(x_grid):
            for j, y in enumerate(y_grid):
                if (i % 2 == 0 and j % 2 == 0) or (i % 2 != 0 and j % 2 != 0):
                    if slope[x, y] <= 15:
                        lon, lat = rasterio.transform.xy(aoi_transform, x, y)
                        if x_min <= lon <= x_max and y_min <= lat <= y_max:  # Check bounds
                            waypoints.append(
                                (lon, lat, terrain[x, y]+1)
                            )

        return waypoints

    @staticmethod
    def redistribute_extreme_slopes(aoi_transform,  AOI_data, slope_data, threshold=15, kernel_size=3):
        """
        Uniformly distribute the elevation values of points with extreme slopes to neighboring areas.

        Parameters:
            elevation_data (numpy.ndarray): The elevation data array.
            slope_data (numpy.ndarray): The slope data array.
            threshold (float): The slope threshold for identifying extreme slopes.
            kernel_size (int): The size of the neighborhood over which to distribute the elevation values.

        """

        elevation_data = AOI_data[0]

        # Get the indices of points with extreme slopes
        extreme_indices = np.where(slope_data < threshold)
        pad_width = kernel_size // 2
        padded_elevation = np.pad(elevation_data, pad_width, mode='mean')

        # Iterate through each extreme point and distribute its elevation
        for index in zip(*extreme_indices):
            i, j = index[0] + pad_width, index[1] + pad_width

            # Extract the neighborhood
            neighborhood = padded_elevation[i - pad_width:i + pad_width + 1, j - pad_width:j + pad_width + 1]

            # Calculate the mean elevation of the neighborhood excluding the center point
            mean_elevation = (np.sum(neighborhood) - padded_elevation[i, j]) / (kernel_size ** 2 - 1)

            # Distribute the elevation value uniformly across the neighborhood
            elevation_data[index[0] - pad_width:index[0] + pad_width + 1,
            index[1] - pad_width:index[1] + pad_width + 1] = mean_elevation

        # Calculate slope with richdem
        terrain = rd.rdarray(elevation_data, no_data=np.nan)
        slope = rd.TerrainAttribute(terrain, attrib='slope_degrees')

        xyz_data = []
        for i in range(0, AOI_data.shape[1]):
            for j in range(0, AOI_data.shape[2]):
                x, y = rasterio.transform.xy(aoi_transform, i, j)
                point = {
                    "Longitude" : x,
                    "Latitude" : y,
                    "elevation": elevation_data[i,j]
                }
                xyz_data.append(
                    point
                )

        optimal_terrain_list = np.array(
            [[key['Longitude'], key['Latitude'], key['elevation']] for key in xyz_data]
        )

        return xyz_data, optimal_terrain_list, elevation_data, slope

    @staticmethod
    def plotting(data, aoi, raw_slope, opt_slope, raw_terrain, ideal_terrain, way_points):
        fig, axs = plt.subplots(1, 5, figsize=(20, 5))  # Changed figsize for better visualization

        # Plot the TIFF file and AOI in the same subplot area
        axs[0].set_title('Original Model with AOI Overlay')
        show(data, ax=axs[0])
        x, y = aoi.exterior.xy
        axs[0].plot(x, y, color='red', linewidth=1)

        axs[1].set_title('Raw Data Slope Analysis')
        axs[1].imshow(raw_slope, cmap='jet')

        axs[2].set_title('Ideal Data Slope Analysis')
        axs[2].imshow(opt_slope, cmap='jet')

        axs[3].set_title('Raw Terrain')
        axs[3].imshow(raw_terrain[0], cmap='Greens')

        axs[4].set_title('Ideal Terrain')
        axs[4].imshow(ideal_terrain, cmap='Greens')
        # Scatter plot for waypoints
        waypoints_x, waypoints_y = zip(*way_points)  # Extract x and y coordinates from list of tuples
        axs[4].scatter(waypoints_y, waypoints_x, color='purple', marker='x')  # Note the order of indexing

        plt.tight_layout()  # Adjust subplot parameters to give specified padding
        plt.show()

    @staticmethod
    def write2csv(dictionary, name):

        # Open the CSV file
        with open(name, 'w', newline='') as csvfile:
            fieldnames = ['Longitude', 'Latitude', 'elevation']

            # Create a CSV writer with space delimiter
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=',', extrasaction='ignore')

            # Write data to the CSV file
            for point in dictionary:
                writer.writerow(point)

        print("\nData appended to CSV file with space delimiters successfully.")


# def main():
#     print("start " + __file__)
#
#     # Importing Dataset (DTM/DEM)
#     dataset = r'D:\Internship\dataextraction\0.5m_DSM\N35721E139508_N35651E139595_UM_DSM_25.tif'
#
#     # Target Coordinates
#     target_coordinates = [35.68476350248405, 139.54965988466046]
#
#     # Getting the bounding box for Area of Interesy, 1 Hectare
#     TIFF_Image, AOI, AOI_data, AOI_transform = create.convert_utm2meter(target_coordinates[0], target_coordinates[1], dataset)
#
#     # Getting the raw terrain and slope data
#     data_slope, data_dict, data_array, optimal_slope, optimal_dict, optimal_array, tree_location = create.extraction(
#         AOI, AOI_data, AOI_transform
#     )
#
#     # Plotting the point clouds
#     create.plotting(TIFF_Image, AOI, raw_slope=data_slope, raw_data=data_array, opt_slope=optimal_slope,
#                     opt_data=optimal_array, waypoints=tree_location)
#
#     # Save to CSV
#     decision = input("Write to CSV?: [Yes/No]")
#     if decision == 'Yes':
#         create.write2csv(data_dict, optimal_dict, tree_location)
#     else:
#         pass
#
#
# if __name__ == '__main__':
#     main()
