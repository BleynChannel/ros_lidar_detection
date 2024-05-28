from rospy import Time
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation
from pyquaternion import Quaternion

class Markers:
	def __init__(self) -> None:
		self.markers = MarkerArray()

	def append_marker(self, center, size, rotation):
		rot = Rotation.from_rotvec(rotation)
		rot_mat = rot.as_matrix()
		orientation = Quaternion(matrix=rot_mat)

		marker = Marker()
		marker.header.frame_id = "/base_link"
		marker.header.stamp = Time.now()
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.lifetime = 10.0
		marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

		marker.pose.position.x = center[0]
		marker.pose.position.y = center[1]
		marker.pose.position.z = center[2]

		marker.pose.orientation = orientation
		
		marker.scale.x = size[0]
		marker.scale.y = size[1]
		marker.scale.z = size[2]

		self.markers.markers.append(marker)
		return marker
	
	def clear(self):
		self.markers.markers = []