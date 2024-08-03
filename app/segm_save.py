import struct
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
import math

rospy.loginfo('Lidar segmentation initializing...')

pc_pub = rospy.Publisher("/pc2_colored", PointCloud2, queue_size=1)

# Initialize global marker array
global_marker_array = None

def marker_callback(marker_array_msg):
    global global_marker_array
    global_marker_array = marker_array_msg.markers

def pointcloud_callback(point_cloud_msg):
    global global_marker_array
    point_generator = pc2.read_points(point_cloud_msg)
    modified_points = []

    for point in point_generator:
        if not math.isnan(point[2]):
            point_inside = False

            if point[2] <= 0.30 and math.sqrt(point[0]**2 + point[1]**2 <= 8):
                r, g, b, a = 0, 255, 0, 255
            elif point[2] <= 0.4:
                r, g, b, a = 255, 255, 0, 255
            elif 0.4 <= point[2] <= 0.7:
                r, g, b, a = 255, 69, 0, 255
            else:
                r, g, b, a = 0, 0, 204, 255

            if (-0.73 <= point[0] <= 2.07) and (-1.16 <= point[1] <= 1) and (point[2] <= 1.6):
                r, g, b, a = 255, 255, 255, 255

                
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            modified_point = [point[0], point[1], point[2], rgb]
            
            if global_marker_array:
                for marker in global_marker_array:
                    if (marker.pose.position.x - marker.scale.x / 2 <= point[0] <= marker.pose.position.x + marker.scale.x / 2 and
                        marker.pose.position.y - marker.scale.y / 2 <= point[1] <= marker.pose.position.y + marker.scale.y / 2 and
                        marker.pose.position.z - marker.scale.z / 2 <= point[2] <= marker.pose.position.z + marker.scale.z / 2):
                        point_inside = True
                        break    
                if point_inside:
                    r, g, b, a = 128, 0, 128, 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    modified_point = [point[0], point[1], point[2], rgb]
            
            modified_points.append(modified_point)

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgba", 12, PointField.UINT32, 1),
    ]

    modified_pc2_msg = pc2.create_cloud(point_cloud_msg.header, fields, modified_points)
    pc_pub.publish(modified_pc2_msg)

rospy.Subscriber("/cloud_concatenated", PointCloud2, pointcloud_callback, queue_size=1)
rospy.Subscriber("/detection/boxs", MarkerArray, marker_callback, queue_size=1)

rospy.loginfo('Lidar segmentation initialized')
