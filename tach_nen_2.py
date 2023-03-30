import open3d as o3d
import numpy as np
import time
import pandas as pd
import cv2
# Đọc file ply chứa point cloud
print("Optimized: ==================")
img = cv2.imread("example_official/demo8_RGB.png")
processing_time = []
# for i in range(20):
t1 = time.time() 
pcd = o3d.io.read_point_cloud("example_official/demo8.ply")
pcd = pcd.voxel_down_sample(voxel_size=0.05)
array_3d = np.asarray(pcd.points)
point_z = array_3d[:,2]
point_z = point_z > -4.0
array_3d = array_3d[point_z]
pcd.points = o3d.utility.Vector3dVector(array_3d)

pcd_normals = o3d.geometry.PointCloud() #create point cloud for normal vectors
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0,0,0]) #create origin axis
t2 = time.time()

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
t10 = time.time()
pcd_normals.points = o3d.utility.Vector3dVector(pcd.normals)

point_in_normal = np.array(pcd_normals.points)
# print(point_in_normal)

index_normal = np.where((abs(point_in_normal[:,1]) < 0.3) & (abs(point_in_normal[:,2]) < 0.3) & (abs(point_in_normal[:,0]) > 0))
# print(len(index_normal[0]))
limit_normal = pcd_normals.select_by_index(index_normal[0])
# print(len(limit_normal.points))

plane_model, inliers = limit_normal.segment_plane(distance_threshold=0.005, #0.01,3,1000
                                        ransac_n=3,
                                        num_iterations=100)


pair = []
plane_on_pcd = []
a = 0
for i in range(len(limit_normal.points)):
    pair.append([index_normal[0][i], i])
    if pair[i][1] == inliers[a]:
        plane_on_pcd.append(pair[i][0])
        a += 1

t11 = time.time()

segmented_plane = pcd.select_by_index(plane_on_pcd)
plane_in_normal = pcd_normals.select_by_index(plane_on_pcd)

t4 = time.time()
#RGB
pcd_normals.paint_uniform_color([1,0,0]) #red
segmented_plane.paint_uniform_color([1,0,1]) #purple
plane_in_normal.paint_uniform_color([1,1,0]) #yellow
limit_normal.paint_uniform_color([0,1,0]) #green

o3d.visualization.draw_geometries([frame,pcd_normals,segmented_plane, limit_normal,plane_in_normal,pcd])

# print("Okkkkkkkkkkkkkkkkkkkk!")
# print(round(t4-t2,3))
# print(t11-t10)


print("pcd:",np.array(pcd.points).shape)
print("plane:",np.array(segmented_plane.points).shape)
fx = 608.6419
fy = 608.6419
# fx = 600
# fy = 600
Cx = 424/2
Cy = 240/2

# print(np.array(segmented_plane.points))


# print(plane_on_pcd)
count = 0
t = np.array(pcd.points)
for i in plane_on_pcd:
    x = int(fx*abs(t[i][0]/t[i][2]) + Cx)
    y = int(fy*abs(t[i][1]/t[i][2]) + Cy)

    yr = x
    xr = 240 - y

    # print(xr,yr)
    try:
        img[yr,xr] = 255
        count += 1
    except:
        pass
print(count)
cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()