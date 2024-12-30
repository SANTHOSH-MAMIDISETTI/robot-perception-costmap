import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
#  the code took a while to run so i added some debug prints to see where the code is stuck( if it is stuck)
def filter_elevation(points: np.ndarray, low: float, high: float) -> np.ndarray:
    """Filter the points based on elevation thresholds."""
    print(f"Debug: Filtering points with elevation between {low} and {high}")
    filtered_points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
    print(f"Debug: {len(filtered_points)} points remain after filtering")
    return filtered_points

def normalize(cost: list) -> np.ndarray:
    """Normalize cost values to range [0, 1]."""
    print("Debug: Normalizing cost values")
    cost_min = np.min(cost)
    cost_max = np.max(cost)
    print(f"Debug: Cost min: {cost_min}, Cost max: {cost_max}")
    normalized_cost = (cost - cost_min) / (cost_max - cost_min) if cost_max > cost_min else cost
    print("Debug: Cost normalization complete")
    return normalized_cost

def cost_map(pointcloud: o3d.geometry.PointCloud, ele_max: float, ele_min: float, nn: int, max_points: int = 1000) -> np.ndarray:
    """Generate a cost map from the point cloud based on elevation gradients."""
    print("Debug: Generating cost map")
    points = np.asarray(pointcloud.points)
    print(f"Debug: Point cloud has {len(points)} points")

    filtered_points = filter_elevation(points, ele_min, ele_max)
    
    if len(filtered_points) == 0:
        print("Debug: No points left after filtering, returning empty array")
        return np.array([])
    
    print("Debug: Building KDTree with filtered points")
    tree = KDTree(filtered_points[:, :2])  # Using x, y for KDTree

    cost = []
    point_count = 0
    for p in filtered_points:
        if point_count >= max_points:
            print("Debug: Reached maximum point limit, stopping loop")
            break

        neighbors_idx = tree.query(p[:2], k=nn)[1]  # Get indices of nearest neighbors
        neighbors = filtered_points[neighbors_idx]
        print(f"Debug: Found {len(neighbors)} neighbors for point {p[:2]}")

        gradient = []
        for n in neighbors:
            x_grad = (n[2] - p[2]) / (n[0] - p[0]) if p[0] != n[0] else 0  # to Avoid division by zero
            y_grad = (n[2] - p[2]) / (n[1] - p[1]) if p[1] != n[1] else 0  # to Avoid division by zero

            magnitude = np.sqrt(x_grad ** 2 + y_grad ** 2)
            gradient.append(magnitude)
        
        median_gradient = np.median(gradient)
        print(f"Debug: Median gradient for point {p[:2]} is {median_gradient}")
        cost.append(median_gradient)

        point_count += 1

    print("Debug: Finished processing points, starting normalization")
    return normalize(np.array(cost))
