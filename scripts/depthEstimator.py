import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from PIL import Image
import torch
from transformers import GLPNImageProcessor, GLPNForDepthEstimation
import numpy as np
import open3d as o3d

def depth_main(centroids: list, yolo_frame: np.ndarray):
    feature_extractor = GLPNImageProcessor.from_pretrained("vinvino02/glpn-nyu")
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = GLPNForDepthEstimation.from_pretrained("vinvino02/glpn-nyu")

    # image = Image.open("/ws/rb5_vision/images/image.jpg")
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
        
    # print(image.size)   
    pad = 16
    output = predicted_depth.squeeze().cpu().numpy() * 1000 
    output = output[pad:-pad, pad:-pad]
    image = image.crop((pad, pad, image.width - pad, image.height - pad))

    fig, ax = plt.subplots(1,2)
    ax[0].imshow(image)
    ax[0].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    ax[1].imshow(output, cmap="plasma")
    ax[1].tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    plt.tight_layout()
    plt.show()

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

    centroids_l = centroids
    depth_output = predicted_depth[pad:-pad, pad:-pad]
    h, w = image.shape[:2]  # shape gives (height, width, channels) for RGB images
    for i, (cx, cy) in enumerate(centroids_l):
        scale_x = new_w / w
        scale_y = new_h / h
        cx_resized = int(cx * scale_x) - pad
        cy_resized = int(cy * scale_y) - pad

        # Ensure the adjusted centroid is within bounds
        if 0 <= cx_resized < depth_output.shape[1] and 0 <= cy_resized < depth_output.shape[0]:
            depth_value = depth_output[cy_resized, cx_resized]
            print(f"Object {i+1} Depth at centroid ({cx}, {cy}): {depth_value} mm")
        else:
            print(f"Object {i+1} Centroid ({cx}, {cy}) is out of bounds after resizing and padding.")
    # o3d.visualization.draw_geometries([pcd])

    
