#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


HEIGHT = 60
WIDTH = 60
RESOLUTION = 0.1

class input_occ_grid:
	def __init__(self):
		self.costmap = 0
		self.quaternion = Quaternion()
		self.point = Point()
		

	def callback(self,data):
		self.costmap = data.data
		self.quaternion = data.info.origin.orientation
		self.point = data.info.origin.position
		
class stacked_costmap:
	def __init__(self):
		self.HEIGHT = 60
		self.WIDTH = 60
		self.RESOLUTION = 0.1
		self.point = Point()
	
	def pose_callback(self,data):
		self.point = data.pose.pose.position

	def run(self,costmap_topic):
		rospy.init_node("stacked_local_occupancy_grid")
		pub = rospy.Publisher("map/stacked_local_occupancy_grid", OccupancyGrid, queue_size=5, latch=True)
		pose_sub = rospy.Subscriber("odometry/filtered_map_baselink", Odometry, self.pose_callback)
 
		occ_grid = [None]*len(costmap_topic)
		sub = [None]*len(costmap_topic)
		for i in range(len(costmap_topic)):
			occ_grid[i] = input_occ_grid()				
			sub[i] = rospy.Subscriber(costmap_topic[i], OccupancyGrid, occ_grid[i].callback)
		
		
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			map_img = np.zeros([HEIGHT,WIDTH,1], \
					    dtype=np.uint8)
			map_img = map_img.flatten()
			map_img = map_img.tolist()
			
			for i in range(len(costmap_topic)):
				map_img = np.add(map_img, occ_grid[i].costmap)
			
			occupancy_grid = np.clip(map_img,0,100)

			map_msg = OccupancyGrid()

			map_msg.header = Header()
			map_msg.header.frame_id = "map"
			map_msg.header.stamp    = rospy.Time.now()

			map_msg.info= MapMetaData()
			map_msg.info.map_load_time = rospy.Time.now()
			map_msg.info.height = self.HEIGHT      #Unit: Pixel
			map_msg.info.width  = self.WIDTH      #Unit: Pixel
			map_msg.info.resolution = self.RESOLUTION

			map_msg.info.origin = Pose()
			map_msg.info.origin.position = Point()
			map_msg.info.origin.position.x = self.point.x - self.WIDTH*self.RESOLUTION/2      #Unit: Meter
			map_msg.info.origin.position.y = self.point.y - self.HEIGHT*self.RESOLUTION/2      #Unit: Meter
			map_msg.info.origin.position.z = 0
			map_msg.info.origin.orientation = Quaternion()
			map_msg.info.origin.orientation.x = 0
			map_msg.info.origin.orientation.y = 0
			map_msg.info.origin.orientation.z = 0
			map_msg.info.origin.orientation.w = 1

			map_msg.data.extend(occupancy_grid)

			pub.publish(map_msg)
			rate.sleep()
		
		

if __name__ == "__main__":
	try:
		main = stacked_costmap()
		main.run(["move_base/local_costmap/costmap","map/free_local_occupancy_grid"])
	except rospy.ROSInterruptException:
		pass
