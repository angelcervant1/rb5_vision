#!/usr/bin/env python3
import rospy
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from PIL import Image
from depth_estimator.msg import ImageWithCentroids
import torch
from transformers import GLPNImageProcessor, GLPNForDepthEstimation
import numpy as np
import open3d as o3d
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class DepthEstimator():
    def __init__(self):
        rospy.init_node("depth_estimator")
        self.sub = rospy.Subscriber("/yolo_prediction", ImageWithCentroids, self.yolo_msg_cb)
        self.pub = rospy.Publisher("/image/detected_pose", PoseStamped, queue_size=10)
        self.drone_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb)
        self.img = None
        self.drone_x = 0
        self.drone_y = 0
        self.cvb = CvBridge()
        self.intrinsic_matrix = np.array([[279, 0, 345],
                                    [0, 279, 211],
                                    [0, 0, 1]])
        self.local_transformers_path = "/ws/rb5_vision/src/depth_estimator/scripts/models/models--vinvino02--glpn-nyu/snapshots/cb8ce03174424af9a3315da175ead88667f0578c"
        
    def yolo_msg_cb(self, yolo_msg):
        self.img = self.cvb.imgmsg_to_cv2(yolo_msg.image, desired_encoding="bgr8")
        self.centroids = [(centroid.x, centroid.y) for centroid in yolo_msg.centroids]        
        if self.img is not None:
            self.depth_main()
        else:
            rospy.logwarn("Failed to received yolo message..")
            return

    def drone_pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        
    def pixel_to_world(self, u, v, Z):
        
        # Extract intrinsic parameters
        fx, fy = 279, 279
        cx, cy = 345, 211

        # Convert pixel coordinates to camera coordinates
        X_c = (u - cx) * Z / fx
        Y_c = (v - cy) * Z / fy
        Z_c = Z
       
        return X_c, Y_c, Z_c
    
    def depth_main(self):
        curr_time = rospy.Time.now()    
        yolo_frame = self.img 
        feature_extractor = GLPNImageProcessor.from_pretrained(self.local_transformers_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model = GLPNForDepthEstimation.from_pretrained(self.local_transformers_path)

        if yolo_frame is not None:
            image = Image.fromarray(yolo_frame) if isinstance(yolo_frame, np.ndarray) else yolo_frame
            new_h = 480 if image.height < 480 else image.height
            new_h -= (new_h % 32)
            new_w = int(new_h * image.width/image.height)
            diff = new_w % 32

            new_w = new_w - diff if diff < 16 else new_w + 32 - diff
            new_size = (new_w, new_h)
            image = image.resize(new_size)

            inputs = feature_extractor(images=image, return_tensors="pt")

            with torch.no_grad():
                outputs = model(**inputs)
                predicted_depth = outputs.predicted_depth
                
            # rospy.loginfo(f"Predicted Depth Shape: {predicted_depth.shape}")
            
            # Check if predicted depth is valid
            # if predicted_depth.shape[1] == 0 or predicted_depth.shape[2] == 0:
            #     rospy.logwarn("Predicted depth is empty. Skipping further processing.")
            #     return  
            # print(image.size)   
            pad = 16
            output = predicted_depth.squeeze().cpu().numpy() * 1000 
            output = output[pad:-pad, pad:-pad]
            image = image.crop((pad, pad, image.width - pad, image.height - pad))

            # fig, ax = plt.subplots(1,2)
            # ax[0].imshow(image)
            # ax[0].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
            # ax[1].imshow(output, cmap="plasma")
            # ax[1].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
            # plt.tight_layout()
            # plt.show()

            width, height = image.size
            depth_img = (output * 255 / np.max(output)).astype('uint8')  
            image = np.array(image)

            depth_o3d = o3d.geometry.Image(depth_img)
            image_o3d = o3d.geometry.Image(image)
            rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(image_o3d, depth_o3d, convert_rgb_to_intensity=False)

            camera_intrinsic = o3d.camera.PinholeCameraIntrinsic()
            camera_intrinsic.set_intrinsics(width, height, 500, 500, width/2, height/2)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img, camera_intrinsic)

            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=14, std_ratio=6.0)
            pcd = pcd.select_by_index(ind)

            pcd.estimate_normals()
            pcd.orient_normals_to_align_with_direction()

            object_pose = PoseStamped()
            object_pose.header.stamp = curr_time
            object_pose.header.frame_id = "map"
            
            depth_output = output
            
            h, w = image.shape[:2]  # shape gives (height, width, channels) for RGB images
            for i, (cx, cy) in enumerate(self.centroids):
                scale_x = new_w / w
                scale_y = new_h / h
                cx_resized = int(cx * scale_x) - pad
                cy_resized = int(cy * scale_y) - pad
                # rospy.loginfo(f"Centroid Resized: {cx_resized}, {cy_resized}")
                # rospy.loginfo(f"Depth Output{depth_output.shape}")
                # Ensure the adjusted centroid is within bounds
                if 0 <= cx_resized < depth_output.shape[1] and 0 <= cy_resized < depth_output.shape[0]:
                    # print("World Coordinates:", world_coords)
                    depth_value = depth_output[cx_resized, cy_resized] / 1000.0 # From mm to m
                    world_coords = self.pixel_to_world(cx_resized, cy_resized, depth_value)
                    object_pose.pose.position.x = self.drone_x + world_coords[0]
                    object_pose.pose.position.y = self.drone_y + world_coords[2]
                    object_pose.pose.position.z = 0
                    self.pub.publish(object_pose)
                    rospy.loginfo(f"Object {i+1} Depth at centroid ({cx}, {cy}): {depth_value} m")
                else:
                    rospy.loginfo(f"Object {i+1} Centroid ({cx}, {cy}) is out of bounds after resizing and padding.")
            # o3d.visualization.draw_geometries([pcd])
        else:
            rospy.logwarn(f"Depth Estimator has no input image")
        
        
if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            de = DepthEstimator()            
            rospy.spin()
        except rospy.ROSInterruptException as e:
            print(e)
    
