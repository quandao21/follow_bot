import numpy as np
import open3d as o3d
import copy
import time

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0, 0, 1])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


# print("Load a ply point cloud, print it, and render it")
# pcd_point_cloud = o3d.data.PCDPointCloud()
# pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

pcd = o3d.io.read_point_cloud('example_official/demo4.ply')

# ply_point_cloud = o3d.data.PLYPointCloud()
# pcd = o3d.io.read_point_cloud(ply_point_cloud.path)

axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
# # print(pcd)
# # print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# print("Downsample the point cloud with a voxel of 0.05")
pcd = pcd.voxel_down_sample(voxel_size=0.01)
# o3d.visualization.draw_geometries([downpcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

print("Recompute the normal of the downsampled point cloud")
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
t1= time.time()
# -- Find point clouds have normal vector parallel with z axis
nor_vector = np.asarray(pcd.normals)
# print(nor_vector[:20, :])
orient_y = pcd.select_by_index(np.where((-1 < nor_vector[:, 1]) & (nor_vector[:, 1] < -0.99))[0])
orient_y.paint_uniform_color([1.0, 0, 0])
print(orient_y)

# -- Rotation
# tf = copy.deepcopy(axis_pcd).translate((0, 0, 0))
# R = axis_pcd.get_rotation_matrix_from_xyz((0, np.pi, 0))
# axis_pcd.rotate(R, center=(0,0,0))

# -- Translate
# tl_z = copy.deepcopy(orient_z).translate((0, -2.3, 0))

# -- Plane segmentation: find the plane with the largest support in the point cloud
plane_model, inliers = orient_y.segment_plane(distance_threshold = 0.01,
                                              ransac_n = 3,
                                              num_iterations = 1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
floor = orient_y.select_by_index(inliers)
floor.paint_uniform_color([1.0, 1.0, 0])
outlier_cloud = orient_y.select_by_index(inliers, invert=True)
t2= time.time()
# -- Outlier removal
# print("Radius oulier removal")
# cl, ind = floor.remove_statistical_outlier(nb_neighbors=20,
#                                             std_ratio=1.0)
# outlier = floor.select_by_index(ind)
# outlier.paint_uniform_color([0, 1.0, 1.0])
# display_inlier_outlier(floor, ind)
print(round(t2-t1,5))
o3d.visualization.draw_geometries([floor] + [axis_pcd],
                                    point_show_normal=False)
# # o3d.visualization.draw_geometries([orient],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024],
#                                   point_show_normal=False)
