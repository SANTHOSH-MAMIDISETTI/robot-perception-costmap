#!/usr/bin/env python

#  the code took a while to run so i added some debug prints to see where the code is stuck( if it is stuck)
import rospy
import numpy as np
import open3d as o3d
from nav_msgs.msg import OccupancyGrid
import yaml
import os
import rospkg
from cost_mapping import cost_map

def load_params():
    """Load parameters from the YAML file located in the package directory."""
    print("Debug: Starting to load parameters...")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('cost-mapping')
    yaml_file_path = os.path.join(package_path, "params.yaml")
    print(f"Debug: Params file path resolved to {yaml_file_path}")

    print(f"Loading params from: {yaml_file_path}")  # Debug print
    
    with open(yaml_file_path, 'r') as file:
        return yaml.safe_load(file)
    print("Debug: Parameters successfully loaded")

def main():
    """Main function for the occupancy node."""
    print("Debug: Initializing ROS node...")
    rospy.init_node('occupancy_node')
    
    # Load parameters
    print("Debug: Loading parameters...")
    params = load_params()
    
    # Use rospkg to get the absolute path to scans.pcd
    print("Debug: Resolving point cloud file path...")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('cost-mapping')
    point_cloud_path = os.path.join(package_path, params['point_cloud_path'])
    
    print(f"Loading point cloud from: {point_cloud_path}")  # Debug print

    # Read the point cloud
    print("Debug: Reading point cloud data...")
    point_cloud = o3d.io.read_point_cloud(point_cloud_path)
    print(f"Point cloud has {len(point_cloud.points)} points")  # Debug print
    print("i guess the file is read")  # Debug print
    # Generate cost map
    print("Debug: Generating cost map...")
    cost_map_array = cost_map(point_cloud, params['elevation_thres_max'], 
                              params['elevation_thres_min'], 
                              params['nearest_neighbors'])
    print(f"Debug: Cost map generated with {len(cost_map_array)} values")
    # Publish occupancy grid
    occupancy_topic = params['occupancy_topic']
    print(f"Debug: Setting up publisher on topic {occupancy_topic}")
    pub = rospy.Publisher(occupancy_topic, OccupancyGrid, queue_size=10)

    # Create occupancy grid message
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    print(f"Resolution: ioasdjiofd")  # Debug print
    occupancy_grid.info.resolution = 1.0  # Adjust as necessary
    occupancy_grid.info.width = len(cost_map_array)
    occupancy_grid.info.height = 1  # This is a 1D array, adjust if needed
    occupancy_grid.data = [-1] * occupancy_grid.info.width  # Unknown initially

    print("Debug: Occupancy grid data being prepared")
    for i, cost in enumerate(cost_map_array):
        occupancy_value = 100 if cost > params['cost_thres'] else 0
        occupancy_grid.data[i] = occupancy_value
    print("Debug: Occupancy grid data prepared")

    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        pub.publish(occupancy_grid)
        print("Debug: Published occupancy grid data")
        rate.sleep()

if __name__ == "__main__":
    print("Debug: Starting main function")
    main()
