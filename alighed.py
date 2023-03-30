import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

pipeline = rs.pipeline()
config = rs.config()
colorizer = rs.colorizer()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

#This code checks if the device has an RGB camera sensor. 
# If not, it prints an error message and exits.
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)


config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)


align_to = rs.stream.color
align = rs.align(align_to)
pc = rs.pointcloud()
count_1 = 8
# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        colorized = colorizer.process(frames)


        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        
        color_frame = aligned_frames.get_color_frame()
        # color_frame=  cv2.rotate(color_frame,cv2.ROTATE_90_COUNTERCLOCKWISE)

        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_image=  cv2.rotate(depth_image,cv2.ROTATE_90_COUNTERCLOCKWISE)
        color_image =  cv2.rotate(color_image,cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        depth_image = depth_image * depth_scale
        depth_save = np.empty((depth_image.shape[0],depth_image.shape[1]))

        # cv2.normalize(depth_image, depth_save, 0, 255, cv2.NORM_MINMAX)        
        print("==========================================")
        print(depth_save)
        print("******************************************")



        # depth_image = cv2.rotate( cv2.flip(depth_image, 0), cv2.ROTATE_90_COUNTERCLOCKWISE)
        # color_image = cv2.rotate( cv2.flip(color_image, 0), cv2.ROTATE_90_COUNTERCLOCKWISE)

        # pcd_1 = o3d.geometry.PointCloud()
        # pcd_1.points = o3d.utility.Vector3dVector(depth_image)


        # o3d.visualization.draw_geometries([pcd_1])

        

        cv2.imshow('Color image', color_image)
        cv2.imshow('Depth image', depth_image)
        # ply = rs.save_to_ply("cam.ply")


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if cv2.waitKey(1) & 0xFF == ord('a'):
            ply = rs.save_to_ply("example_official/demo"+str(count_1)+".ply")

            # Set options to the desired values
            # In this example we'll generate a textual PLY with normals (mesh is already created by default)
            ply.set_option(rs.save_to_ply.option_ply_binary, False)
            ply.set_option(rs.save_to_ply.option_ply_normals, True)

            print("Saving to demo.ply..." +str(count_1))
            # Apply the processing block to the frameset which contains the depth frame and the texture
            ply.process(colorized)
            cv2.imwrite("example_official/demo"+str(count_1)+"_RGB.png", color_image)
            cv2.imwrite("example_official/demo"+str(count_1)+"_Depth.png", depth_save)
            print("Done")
            count_1 += 1
            
finally:
    pipeline.stop()

