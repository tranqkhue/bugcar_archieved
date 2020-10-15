import cv2
import rospy
import numpy as np
from numpy import sin,cos
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


MAP_RESOLUTION = 0.1  #Unit: Meter
MAP_SIZE       = 20   #Unit: Meter, Shape: Square with center "base_link"
MAP_SIZE_PIXEL = int(MAP_SIZE / MAP_RESOLUTION)

map_topic = "map/lidar_occupancy_grid"
OG_publisher = rospy.Publisher(map_topic, OccupancyGrid, queue_size=5, latch=True)




def scan_to_og(data):
	img = np.ones(shape = (MAP_SIZE_PIXEL,MAP_SIZE_PIXEL))*100
	print(img.shape)
	polygon = []
	origin = np.array([int(MAP_SIZE_PIXEL/2),int(MAP_SIZE_PIXEL/2)])
	# remember, the origins in occupancy_grid and img are different. ( one is topleft, the other is  in the botleft corner)
	# so we have to flip the image vertically in order to match the occupancy_grid origin with the image origin.
	for enum,distance in enumerate(data.ranges):
		#print(distance)
		angle = data.angle_increment * enum  # in radians

		#it would be so computationally expensive to flip the image,so we only need to flip the vertices of the filled polygon
		#which will achieve the same result.
		cartesian_coordinate = np.array([distance*sin(angle),-distance*cos(angle)]) # in metres
		pixel_coordinate = cartesian_coordinate / MAP_RESOLUTION
		polygon.append(pixel_coordinate+ origin)
	polygon = np.asarray(polygon).astype(np.int32)
	img = cv2.fillPoly(img,[polygon],(0,0,0),cv2.LINE_AA).astype(np.uint8)
	cv2.imshow("img",img)
	cv2.waitKey(1)
	map_msg = create_occ_grid(img,origin)
	OG_publisher.publish(map_msg)



def create_occ_grid(map_img,origin):
	occupancy_grid = map_img.flatten()
	occupancy_grid = occupancy_grid.tolist()

	map_msg = OccupancyGrid()

	map_msg.header = Header()
	map_msg.header.frame_id = "base_link"
	map_msg.header.stamp    = rospy.Time.now()

	map_msg.info= MapMetaData()
	map_msg.info.map_load_time = rospy.Time.now()
	map_msg.info.height = MAP_SIZE_PIXEL      #Unit: Pixel
	map_msg.info.width  = MAP_SIZE_PIXEL      #Unit: Pixel
	map_msg.info.resolution = MAP_RESOLUTION

	map_msg.info.origin = Pose()
	map_msg.info.origin.position = Point()
	map_msg.info.origin.position.x = origin[0]      #Unit: Meter
	map_msg.info.origin.position.y = origin[1]      #Unit: Meter
	map_msg.info.origin.position.z = 0
	map_msg.info.origin.orientation = Quaternion()
	map_msg.info.origin.orientation.x = 0
	map_msg.info.origin.orientation.y = 0
	map_msg.info.origin.orientation.z = 0
	map_msg.info.origin.orientation.w = 1

	map_msg.data.extend(occupancy_grid)
	return map_msg

rospy.init_node('listener')
rospy.Subscriber('/scan',LaserScan,scan_to_og)
rospy.spin()
