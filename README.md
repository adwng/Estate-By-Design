# ESTATE BY DESIGN

In this repository, you will find a number of files related to Estate By Design:
> [!IMPORTANT]
> Performance and algorithms can be improved with using Eigen and PCL libraries, please consider migrating to C++ for continuous development

| Files |Description  |
|--|--|
|docs|Consists a paper for transitional A* search|
|APF_3D|Library for Transitional A star Search and Utilities - v2|
|AstarFinder3D|Library for A star Search and Utilities - v1|
|Dynamic Astar|A test bed for transitional A*|
|Goole_Earth_Engine_API_Test|A test during early development for using the google earth engine API instead of DTM from vendors|
|PerlinNoise|A library to generate procedural height maps|
|estate_by_design_v2|Uses APF_3D to compute|
|estate_under_design|Uses AstarFinder3D to compute|

## Usage

 ### Estate By Design V2
> [!TIP]
> To use DTM data for testing algorithms on terrestrial data

    path_points = []
     
    dataset = r'D:\Internship\dataextraction\0.5m_DSM\N35721E139508_N35651E139595_UM_DSM_25.tif'
    
    target_coordinates = [35.68476350248405, 139.54965988466046]
    
    TIFF_Image, AOI, AOI_data, AOI_transform = create.convert_utm2meter(
            target_coordinates[0], target_coordinates[1], dataset
        )
    
    data_slope, data_dict, data_array = create.extraction(AOI_data, AOI_transform)
    
    optimal_terrain_dict, optimal_terrain, elevation_data, optimal_slope = create.redistribute_extreme_slopes(
            AOI_transform, AOI_data,
            data_slope, threshold=15,
            kernel_size=9)
> [!TIP]
> To use Perlin noise generation for procedural terrain generation for use of algorithm testing.

 - [x] Comment the following code block above in the code
 - [x] Uncomment 
      ```
      generator = GenerateHeightMap()
      elevation_data, grid, optimal_slope = generator.generate()
      ```

## TODO

 - [ ] Improve cut and fill algorithm
 - [ ] Split path searching algorithms into multiple finders given one hectare of field
 - [ ] Improve sorting algorithm for waypoints
 - [ ] Add path smoothing to smooth paths
 - [ ] Add functionality for recognizable obstacles

```
