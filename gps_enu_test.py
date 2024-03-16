from gps_utils import GPS_utils
import matplotlib.pyplot as plt
import csv

gps = GPS_utils()

csv_file_path = 'coords.csv'
csv_data = []
xCoords = []
yCoords = []

with open(csv_file_path, newline='') as csvfile:
    csv_reader = csv.DictReader(csvfile)
    for row in csv_reader:
        csv_data.append(row)
    
gps.setENUorigin(float(csv_data[0]['lat']), float(csv_data[0]['long']), float(csv_data[0]['height']))

for row in csv_data[1:]:
    lat = float(row['lat'])
    long = float(row['long'])
    height = float(row['height'])

    enu = gps.geo2enu(lat, long, height)

    # print([enu.item(0),enu.item(1)])

    xCoords.append(enu.item(0))
    yCoords.append(enu.item(1))

for i in range(len(xCoords)):
    enu = gps.enu2geo(xCoords[i], yCoords[i], 0)
    print([enu.item(0), enu.item(1)])

# Plot the points
plt.scatter(xCoords, yCoords, color='blue')

# Customize the plot (optional)
plt.title('Scatter Plot of Points')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()

# Display the plot
plt.grid(True)
plt.show()
