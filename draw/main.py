import requests
import plotly.graph_objects as go
import os

# requests.get('http://localhost:8080/searchpath?x1=144.9631&y1=-37.8136&x2=145.0458&y2=-37.8768').json()
response  = requests.get('http://localhost:8080/searchpath', params={'x1': 144.9631, 'y1': -37.8136, 'x2': 145.0458, 'y2': -37.8768})
data = response.json()
path = data['path']

# Get Mapbox Access Token from .env
with open('.env', 'r') as f:
    for line in f:
        if line.startswith('MAPBOX_TOKEN='):
            MAPBOX_TOKEN = line.split('=')[1].strip()

# Create a figure with Mapbox
fig = go.Figure()

# Iterate through the path list and plot each edge as a line on the map
points_full = []
for i, edge in enumerate(path):
    points = edge['geom']
    if points[-1] == path[-1]['geom'][0]:
        points_full += points[:-1]
    else:
        points_full += points

# Separate longitude and latitude for x and y coordinates
lon_coords = [point[0] for point in points_full]
lat_coords = [point[1] for point in points_full]
    
# Add the path as a line trace on the map
fig.add_trace(go.Scattermapbox(
    mode='lines+markers',
    lon=lon_coords,
    lat=lat_coords,
    line=dict(color='blue', width=2),  # Customize the line style
    marker=dict(size=2, color='red'),  # Customize the markers
))

# Update layout with Mapbox style and access token
fig.update_layout(
    mapbox=dict(
        accesstoken=MAPBOX_TOKEN,
        center=dict(lon=144.9631, lat=-37.8136),  # Set the center of the map
        zoom=11,  # Set zoom level (adjust as needed)
        style="streets"
    ),
    # title="Path on Mapbox"
)

# # Allow the map to be zoomed in and out
fig.update_layout(margin={"r":2,"t":2,"l":2,"b":2})

# Save the map to an HTML file
fig.write_html("path_on_mapbox.html")