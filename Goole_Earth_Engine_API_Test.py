import ee
import csv
from pyproj import Proj

pp = Proj(proj='utm',zone=47,ellps='WGS84', preserve_units=False)

service_account = 'andrew-ng@ee-zacklen12345.iam.gserviceaccount.com'
credentials = ee.ServiceAccountCredentials(service_account, 'ee-zacklen12345-852e129a1e90.json')

ee.Initialize(credentials)

# Define the coordinate of interest (latitude, longitude)
coordinate = ee.Geometry.Point(5.80258, 51.78547)


# Define the region of interest as a one-hectare area around the coordinate
region_of_interest = coordinate.buffer(50).bounds()

# USGS/SRTMGL1_003

# A digital elevation model
elev = ee.Image('AHN/AHN2_05M_NON').clip(region_of_interest).select('elevation')
# Calculate slope. Units are degrees, range is [0,90).
slope = ee.Terrain.slope(elev)

# Sample the elevation and slope data at each pixel within the region of interest
elevation_values = elev.sample(region=region_of_interest, geometries=True, scale=10)
slope_values = slope.sample(region=region_of_interest, geometries=True, scale=10)

# Extract coordinates, elevation, and slope data and save them into an array
elevation_data = elevation_values.getInfo()['features']
slope_data = slope_values.getInfo()['features']

data_array = []
terrainData_array = []

for elevation_feature, slope_feature in zip(elevation_data, slope_data):
    elevation_coords = elevation_feature['geometry']['coordinates']
    lon, lat = elevation_coords
    longitude, latitude = pp(lon, lat)
    elevation_value = elevation_feature['properties']['elevation']
    slope_value = slope_feature['properties']['slope']

    data_array.append({'Longitude': longitude, 'Latitude': latitude, 'elevation': elevation_value})

    terrainData_array.append({'elevation': elevation_value, 'slope': slope_value})

# # Print each data point one by one
# for data_point in data_array:
#     print("Coordinates:", data_point['Longitude'], data_point['Latitude'])
#     print("Elevation:", data_point['elevation'])
#     print("-----------------------------------------")

print("Length of the Array is:", len(data_array))


##Open the CSV file in append mode
with open('data.csv', 'w', newline='') as csvfile:
    # Define field names
    fieldnames = ['Longitude', 'Latitude', 'elevation']

    # Create a CSV writer with space delimiter
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=',')

    # Write data to the CSV file
    for data_point in data_array:
        writer.writerow(data_point)

print("Data appended to CSV file with space delimiters successfully.")




