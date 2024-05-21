from mmcv import Config
from mmdet3d.apis import init_model
import torch
import plotly.graph_objects as go
import open3d as o3d
import numpy as np
import os
import time
from threading import Thread

from scipy.spatial.transform import Rotation
from pyquaternion import Quaternion

from src.utils.inference import inference_model
from src.utils.bounding_box import BoundingBox
import src.models.centerpoint

# config
cfg = Config.fromfile("configs/config.py")
cfg.data.test.box_type_3d = "lidar"
device = torch.device("cuda:0") if torch.cuda.is_available() else "cpu"
model = init_model(cfg, "checkpoints/weights.pth", device)
model_name = "CenterPoint"

labels = cfg["class_names"]
gt_index_to_labels = dict(enumerate(labels))

local_pointcloud_path = 'pcds/'

pcd_paths = []
for file in os.listdir(local_pointcloud_path):
    if file.endswith(".pcd"):
        pcd_paths.append(local_pointcloud_path + file)

def extract_xyz(pcd_file):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2] 
    return points, x, y, z

def get_box_vertexes(center, size, rotation):
    rot = Rotation.from_rotvec(rotation)
    rot_mat = rot.as_matrix()
    orientation = Quaternion(matrix=rot_mat)
    bbox = BoundingBox(center, size, orientation)
    return bbox.corners()

if __name__ == "__main__":
    for pcd_path in pcd_paths:
        points, pcd_x, pcd_y, pcd_z = extract_xyz(pcd_path)

        start = time.time()
        results = inference_model(model, points, gt_index_to_labels, model_name)
        print(f'Inference time = {time.time() - start}s')

    boxes_data = []
    for result in results:
        box_x, box_y, box_z = get_box_vertexes(
            center=result["translation"],
            size=result["size"],
            rotation=[0, 0, result["rotation"] + (np.pi / 2)],
        )
        box_viz = go.Mesh3d(
            x=box_x,
            y=box_y,
            z=box_z,
            i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
            j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
            k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
            opacity=0.3,
            color='#DC143C',
            flatshading = True
        )
        boxes_data.append(box_viz)

    boxes_data.insert(0, go.Scatter3d(
            x=pcd_x,
            y=pcd_y,
            z=pcd_z,
            mode='markers',
            marker=dict(
                size=2,
                opacity=0.8
            )))

    # visualize inference results
    fig = go.Figure(data=boxes_data)
    fig.update_layout(
        margin=dict(l=0, r=0, b=0, t=0),
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z',
            xaxis=dict(range=[-100, 100]),
            yaxis=dict(range=[-80, 80]),
            zaxis=dict(range=[-30, 30])
        ),
    )   
    fig.show()