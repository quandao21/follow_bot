import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import time

def plane_segment(depth_image, color_image, pcd):
    depth_image = np.uint16(depth_image)
    depth_intrinsics = rs.video_stream_profile(aligned_depth_frame.profile).get_intrinsics()
    pcd.points = o3d.geometry.PointCloud.create_from_depth_image(depth=o3d.geometry.Image(depth_image),intrinsic=o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height,depth_intrinsics.fx, depth_intrinsics.fy,depth_intrinsics.ppx, depth_intrinsics.ppy)).points
    pcd = pcd.voxel_down_sample(voxel_size=0.05)
    array_3d = np.asarray(pcd.points)

    point_z = array_3d[:,2]
    point_z = point_z < 4.0
    array_3d = array_3d[point_z]
    pcd.points = o3d.utility.Vector3dVector(array_3d)

    pcd_normals = o3d.geometry.PointCloud()

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50))

    pcd_normals.points = o3d.utility.Vector3dVector(pcd.normals)

    point_in_normal = np.array(pcd_normals.points)

    index_normal = np.where((abs(point_in_normal[:,1]) < 0.3) & (abs(point_in_normal[:,2]) < 0.3) & (abs(point_in_normal[:,0]) > 0))

    limit_normal = pcd_normals.select_by_index(index_normal[0])
    # plane_model, inliers = pcd_normals.segment_plane(distance_threshold=0.01, ransac_n=3,num_iterations=10000)

    plane_model, inliers = limit_normal.segment_plane(distance_threshold=0.005, ransac_n=3,num_iterations=100)


    pair = []
    plane_on_pcd = []
    a = 0
    for i in range(len(inliers)):
        pair.append([index_normal[0][i], i])
        if pair[i][1] == inliers[a]:
            plane_on_pcd.append(pair[i][0])
            a += 1
    plane_pcd = pcd.select_by_index(plane_on_pcd)
    print(np.array(plane_pcd.points).shape)
    print(len(inliers))


    fx = 608.6419
    fy = 608.6419
    Cx = 640/2
    Cy = 360/2

    count = 0
    t = np.array(plane_pcd.points)
    for point in t:
        x = int(fx*(point[0]/abs(point[2])) + Cx)
        y = int(fy*(point[1]/abs(point[2])) + Cy)

        try:
            color_image[y,x] = 255
            count += 1
        except:
            pass
    return 

#------------------------INIT--------------------------------
pipeline = rs.pipeline()
config = rs.config()
colorizer = rs.colorizer()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)
pc = rs.pointcloud()


count_frame = 0
pcd = o3d.geometry.PointCloud()
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.9, origin=[0,0,0]) #create origin axis

for i in range(30):
    frames = pipeline.wait_for_frames()

colorized = colorizer.process(frames)
aligned_frames = align.process(frames)

aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image

color_frame = aligned_frames.get_color_frame()
# color_frame=  cv2.rotate(color_frame,cv2.ROTATE_90_COUNTERCLOCKWISE)

depth_image = np.asanyarray(aligned_depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())
# depth_image=  cv2.rotate(depth_image,cv2.ROTATE_90_COUNTERCLOCKWISE)
# color_image =  cv2.rotate(color_image,cv2.ROTATE_90_COUNTERCLOCKWISE)


depth_save = np.empty((depth_image.shape[0],depth_image.shape[1]))
# cv2.normalize(depth_image, depth_save, 0, 255, cv2.NORM_MINMAX)        

#===========================================================================================================================

depth_image = np.uint16(depth_image)
depth_intrinsics = rs.video_stream_profile(aligned_depth_frame.profile).get_intrinsics()
pcd.points = o3d.geometry.PointCloud.create_from_depth_image(depth=o3d.geometry.Image(depth_image),intrinsic=o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height,depth_intrinsics.fx, depth_intrinsics.fy,depth_intrinsics.ppx, depth_intrinsics.ppy)).points
pcd = pcd.voxel_down_sample(voxel_size=0.05)
array_3d = np.asarray(pcd.points)

point_z = array_3d[:,2]
point_z = point_z < 4.0
array_3d = array_3d[point_z]
pcd.points = o3d.utility.Vector3dVector(array_3d)

pcd_normals = o3d.geometry.PointCloud()

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50))

pcd_normals.points = o3d.utility.Vector3dVector(pcd.normals)

point_in_normal = np.array(pcd_normals.points)

index_normal = np.where((abs(point_in_normal[:,1]) < 0.3) & (abs(point_in_normal[:,2]) < 0.3) & (abs(point_in_normal[:,0]) > 0))

limit_normal = pcd_normals.select_by_index(index_normal[0])
# plane_model, inliers = pcd_normals.segment_plane(distance_threshold=0.01, ransac_n=3,num_iterations=10000)

plane_model, inliers = limit_normal.segment_plane(distance_threshold=0.005, ransac_n=3,num_iterations=100)


pair = []
plane_on_pcd = []
a = 0
for i in range(len(inliers)):
    pair.append([index_normal[0][i], i])
    if pair[i][1] == inliers[a]:
        plane_on_pcd.append(pair[i][0])
        a += 1
plane_pcd = pcd.select_by_index(plane_on_pcd)
print(np.array(plane_pcd.points).shape)
print(len(inliers))


fx = 608.6419
fy = 608.6419
Cx = 640/2
Cy = 360/2

count = 0
t = np.array(plane_pcd.points)
for point in t:
    x = int(fx*(point[0]/abs(point[2])) + Cx)
    y = int(fy*(point[1]/abs(point[2])) + Cy)

    try:
        color_image[y,x] = 255
        count += 1
    except:
            pass


plane_on_normal = pcd_normals.select_by_index(inliers)
pcd_normals.paint_uniform_color([1,0,0]) #red
plane_on_normal.paint_uniform_color([0,0,1]) #blue
limit_normal.paint_uniform_color([0,1,0]) #gree
o3d.visualization.draw_geometries([frame , pcd_normals, limit_normal,plane_pcd])

cv2.imshow('Color image', cv2.rotate(color_image,cv2.ROTATE_90_COUNTERCLOCKWISE))
cv2.imshow('Depth image', depth_image)
cv2.waitKey(0)
cv2.destroyAllWindows()



