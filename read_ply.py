import numpy as np
import open3d as o3d

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("examples/demo2.ply")
print(np.asarray(pcd.points).shape)

pcd = pcd.voxel_down_sample(voxel_size=0.03)
# print(np.asarray(pcd.points))
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
# a = np.asarray(pcd.normals)
# print(a.shape)
# np.savetxt("normals.txt", a)





# array_3d = np.asarray(pcd.points)
# # print(array_3d)
# point_plane = array_3d[:,1]
# point_plane = point_plane < 0.1
# # point_plane = point_plane < -0.1
# filted_pointcloud = array_3d[point_plane]
# pcd_filted= o3d.geometry.PointCloud()

# pcd_filted.points = o3d.utility.Vector3dVector(filted_pointcloud)
# pcd_filted.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
# a = np.array(pcd_filted.points) 
# print(a)
# # pcd_filted = pcd_filted.voxel_down_sample(voxel_size=0.05)
# # pcd.paint_uniform_color([1.0, 0, 0]) #red
# pcd_filted.paint_uniform_color([0, 1.0, 1.0]) #blue

o3d.visualization.draw_geometries([pcd])


# print("====================================")
# print(np.asanyarray(pcd.normals))
# print(array_3d.shape)
