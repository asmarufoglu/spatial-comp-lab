import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from pathlib import Path

# Configuration
# -----------------------------------------------------------------------------
@dataclass
class LidarConfig:
    """Pipeline configuration parameters."""
    voxel_size: float = 0.02      # Downsampling grid size
    nb_neighbors: int = 20        # Neighbors for statistical outlier removal
    std_ratio: float = 2.0        # Std deviation threshold for noise filtering
    ransac_dist: float = 0.01     # Max distance for plane fitting
    cluster_eps: float = 0.05     # DBSCAN epsilon (search radius)
    min_points: int = 10          # DBSCAN min points per cluster

# Lidar Processing Pipeline
# -----------------------------------------------------------------------------
class LidarProcessor:
    def __init__(self, config: LidarConfig, output_dir: str = "output"):
        self.cfg = config
        self.output_path = Path(output_dir)
        self.pcd = None
        self.inlier_cloud = None  # Ground plane
        self.outlier_cloud = None # Segmented objects
        
        self.output_path.mkdir(exist_ok=True)

    def load_data(self):
        print("\n[STEP 1] Loading sample data...")
        # Load sample point cloud from Open3D repository
        dataset = o3d.data.PCDPointCloud()
        self.pcd = o3d.io.read_point_cloud(dataset.path)
        
        if not self.pcd.has_points():
            raise ValueError("Failed to load point cloud data.")
            
        print(f" -> Raw point count: {len(self.pcd.points)}")
        return self

    def pre_process(self):
        """Downsamples the point cloud for performance optimization."""
        if self.pcd is None: raise RuntimeError("Data not loaded.")

        print("\n[STEP 2] Downsampling (Voxel Grid)...")
        self.pcd = self.pcd.voxel_down_sample(voxel_size=self.cfg.voxel_size)
        print(f" -> Downsampled count: {len(self.pcd.points)}")
        return self

    def remove_noise(self):
        """Filters noise using Statistical Outlier Removal (SOR)."""
        print("\n[STEP 3] Removing noise...")
        
        cl, ind = self.pcd.remove_statistical_outlier(
            nb_neighbors=self.cfg.nb_neighbors,
            std_ratio=self.cfg.std_ratio
        )
        
        # Calculate reduction metrics
        noise_count = len(self.pcd.points) - len(cl.points)
        reduction = (noise_count / len(self.pcd.points)) * 100
        
        print(f" -> Removed points: {noise_count}")
        print(f" -> Reduction ratio: {reduction:.2f}%")
        
        self.pcd = cl # Update state
        return self

    def segment_ground(self):
        """Segments ground plane using RANSAC algorithm."""
        print("\n[STEP 4] Ground Segmentation (RANSAC)...")
        
        plane_model, inliers = self.pcd.segment_plane(
            distance_threshold=self.cfg.ransac_dist,
            ransac_n=3,
            num_iterations=1000
        )
        
        a, b, c, d = plane_model
        print(f" -> Plane Equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        
        # Separate ground (inliers) and objects (outliers)
        self.inlier_cloud = self.pcd.select_by_index(inliers)
        self.inlier_cloud.paint_uniform_color([1, 0, 0]) # Red for ground
        
        self.outlier_cloud = self.pcd.select_by_index(inliers, invert=True)
        return self

    def cluster_objects(self):
        """Clusters remaining objects using DBSCAN."""
        print("\n[STEP 5] Object Clustering (DBSCAN)...")
        
        labels = np.array(
            self.outlier_cloud.cluster_dbscan(
                eps=self.cfg.cluster_eps,
                min_points=self.cfg.min_points,
                print_progress=False
            )
        )
        
        max_label = labels.max()
        print(f" -> Detected objects: {max_label + 1}")
        
        # Apply colormap
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0 # Set noise to black
        self.outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        
        return self

    def save_results(self):
        """Persists processed clouds to disk."""
        print(f"\n[STEP 6] Saving results to '{self.output_path}'...")
        
        o3d.io.write_point_cloud(str(self.output_path / "ground.pcd"), self.inlier_cloud)
        o3d.io.write_point_cloud(str(self.output_path / "objects.pcd"), self.outlier_cloud)
        print(" -> Save complete.")
        return self

    def visualize(self):
        """Renders the final scene with custom view options."""
        print("\n[VISUALIZE] Rendering scene...")
        
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Lidar Perception Output [Red=X, Green=Y, Blue=Z]", width=1280, height=720)
        
        vis.add_geometry(self.inlier_cloud)
        vis.add_geometry(self.outlier_cloud)
        
        # Render Options
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.05, 0.05, 0.05]) # Dark background
        opt.point_size = 1.0  # High-res look
        opt.light_on = False  # Unlit mode for raw data visualization
        
        # Reference frame
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.5, origin=[0, 0, 0])
        vis.add_geometry(axis)
        

        vis.run()
        vis.destroy_window()
        return self

# Main Execution
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # Initialize config
    config = LidarConfig(
        voxel_size=0.02,
        nb_neighbors=20,
        std_ratio=2.0
    )
    
    # Execute pipeline
    processor = LidarProcessor(config)
    (
        processor
        .load_data()
        .pre_process()
        .remove_noise()
        .segment_ground()
        .cluster_objects()
        .save_results()
        .visualize()
    )