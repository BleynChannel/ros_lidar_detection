from mmcv import Config
from mmdet3d.apis import init_model
import torch
import numpy as np
import time
from pathlib import Path
import argparse

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
import std_msgs.msg as std_msg
import ros_numpy

import open3d as o3d

from src.utils.inference import inference_model
from src.utils.markers import Markers
import src.models.centerpoint

def msgpcd2points(msg):
	points = ros_numpy.numpify(msg)
	points.dtype = np.float32
	points = points.reshape((-1, len(msg.fields)))
	return points

def save_pcd(points):
	pcd = o3d.geometry.PointCloud()
	
	pc = np.zeros((len(points), 3))
	pc[:, 0] = points[:, 0]
	pc[:, 1] = points[:, 1]
	pc[:, 2] = points[:, 2]
	pcd.points = o3d.utility.Vector3dVector(pc)

	path = f'save/{time.time()}.pcd'
	o3d.io.write_point_cloud(path, pcd)
	return path

def read_pcd(path):
	pcd = o3d.io.read_point_cloud(path)
	return np.asarray(pcd.points)

def main(args):
	# ROS init
	rospy.init_node(args.node_name)
	rospy.loginfo('Node is initialized')

	# Config
	rospy.loginfo('Loading config and weight for model...')
	cfg = Config.fromfile(args.config)
	cfg.data.test.box_type_3d = "lidar"
	device = torch.device("cuda:0") if torch.cuda.is_available() else "cpu"
	model = init_model(cfg, args.weight, device)
	model_name = "CenterPoint"

	labels = cfg["class_names"]
	gt_index_to_labels = dict(enumerate(labels))
	
	rospy.loginfo('Config and weight loaded')
	
	# Main block
	markers = Markers()

	pub_boxs = rospy.Publisher('/detection/boxs', MarkerArray, queue_size=args.queue_size)
	pub_people = rospy.Publisher('/detection/people', std_msg.Int8, queue_size=args.queue_size)
	pub_inf_time = rospy.Publisher('/detection/inference_time', std_msg.Float32, queue_size=args.queue_size)

	def receive_pointcloud(msg):
		rospy.loginfo('Message is received')
		points = msgpcd2points(msg)

		# Save pcd files
		if args.save:
			Path("save").mkdir(exist_ok=True)

			path = save_pcd(points)
			points = read_pcd(path)

		start = time.time()
		results = inference_model(model, points, gt_index_to_labels, model_name)
		inf_time = time.time() - start
		rospy.loginfo(f'Inference time = {inf_time}s')
		
		for result in results:
			markers.append_marker(
				center=result["translation"],
				size=result["size"],
				rotation=[0, 0, result["rotation"] + (np.pi / 2)],
			)

		pub_boxs.publish(markers.markers)
		pub_people.publish(len(markers.markers.markers))
		pub_inf_time.publish(inf_time)

		markers.clear()

	rospy.Subscriber(args.topic_cloud, PointCloud2, receive_pointcloud, queue_size=args.queue_size)
	rospy.loginfo('Listening started')

	# Loop
	rospy.spin()

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Lidar Object Detection')
	parser.add_argument('node_name', type=str)
	parser.add_argument('config', type=str)
	parser.add_argument('weight', type=str)
	parser.add_argument('-t', '--topic_cloud', type=str, default='/cloud')
	parser.add_argument('-s', '--save', action="store_true", default=False)
	parser.add_argument('--queue_size', type=int, default=10)
	args = parser.parse_args()

	main(args)