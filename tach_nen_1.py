import open3d as o3d
import numpy as np

import time
import pandas as pd
    # Đọc file ply chứa point cloud
print("Not optimized: ==================")
processing_time = []

for i in range(40):
    t1 = time.time() 
    pcd = o3d.io.read_point_cloud("example_official/demo4.ply")
    pcd = pcd.voxel_down_sample(voxel_size=0.05)
    array_3d = np.asarray(pcd.points)
    point_z = array_3d[:,2]
    point_z = point_z > -4.0
    array_3d = array_3d[point_z]
    pcd.points = o3d.utility.Vector3dVector(array_3d)
    pcd_normals = o3d.geometry.PointCloud()
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0,0,0])
    t2 = time.time()


    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
    pcd_normals.points = o3d.utility.Vector3dVector(pcd.normals)

    t3 = time.time()
    t10 = time.time()
    plane_model, inliers = pcd_normals.segment_plane(distance_threshold=0.005, #0.01,3,1000
                                            ransac_n=3,
                                            num_iterations=100)
    t11 = time.time()

    plane_normal = pcd_normals.select_by_index(inliers)

    pcd_show_plane = pcd.select_by_index(inliers)
    t4 = time.time()


    pcd_normals.paint_uniform_color([1, 0, 0]) #red
    plane_normal.paint_uniform_color([0, 0, 1]) #blue
    pcd_show_plane.paint_uniform_color([1,1,0]) #yellow
    
    print(t11-t10)
    processing_time.append(round(t11-t10, 5))


data = {"ransactime": processing_time}
df = pd.DataFrame(data)
df.to_csv("not_opt.csv", index=True)
time_trung_binh_1 = np.array(processing_time).mean()


import open3d as o3d
import numpy as np
import time
import pandas as pd
# Đọc file ply chứa point cloud
print("Optimized: ==================")
processing_time = []
for i in range(40):
    t1 = time.time() 
    pcd = o3d.io.read_point_cloud("example_official/demo4.ply")
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

    # o3d.visualization.draw_geometries([ frame,pcd_normals,segmented_plane, limit_normal,plane_in_normal])

    # print("Okkkkkkkkkkkkkkkkkkkk!")
    # print(round(t4-t2,3))
    print(t11-t10)


    processing_time.append(round(t11-t10, 5))


data = {"ransactime": processing_time}
df = pd.DataFrame(data)
df.to_csv("opt.csv", index=True)
time_trung_binh_2 = np.array(processing_time).mean()
print("time trung binh 1:", time_trung_binh_1)
print("time trung binh 2:", time_trung_binh_2)
chenh_lech = round(time_trung_binh_1/time_trung_binh_2,3)
print("nhanh gap " +str(chenh_lech)+"lan")
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

run_time = np.arange(0,40)
# Read the CSV file into a DataFrame
df_no_opt = pd.read_csv('not_opt.csv')
df_opt = pd.read_csv('opt.csv')
# print(df_no_opt['ransactime'])
# Create a line plot of the 'threshold' column against the 'precision' column

plt.plot( run_time,df_no_opt['ransactime'],linestyle='-', linewidth=1 , color ="orange" )
plt.plot( run_time,df_opt['ransactime'],linestyle='-', linewidth=1 , color ="red" )

# Set the title and axis labels
plt.title('Ransac time')
plt.xlabel('Step')

plt.ylabel('Time')

# Display the plot
plt.show()
