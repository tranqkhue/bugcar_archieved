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

class input_occ_grid:
	def __init__(self):
		self.costmap = 0		

	def callback(self,data):
		self.costmap = data.data
		
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
		
		#Subscribe to odometry topic to get robot position
		pose_sub = rospy.Subscriber("odometry/filtered_map_baselink", Odometry, self.pose_callback)
 		
		#Subscribe to OccupancyGrid topics
		occ_grid = [None]*len(costmap_topic)
		sub = [None]*len(costmap_topic)
		for i in range(len(costmap_topic)):
			occ_grid[i] = input_occ_grid()				
			sub[i] = rospy.Subscriber(costmap_topic[i], OccupancyGrid, occ_grid[i].callback)
		
		
		rate = rospy.Rate(50)

		while not rospy.is_shutdown():
			
			#Stacking OccupancyGrids
			occupancy_grid = [0]*(self.WIDTH*self.HEIGHT)			
			for i in range(len(costmap_topic)):
				occupancy_grid = np.add(occupancy_grid, occ_grid[i].costmap)
			
			#Limit value 0 - 100 
			occupancy_grid = np.clip(occupancy_grid,0,100)

			map_msg = OccupancyGrid()

			map_msg.header = Header()
			map_msg.header.frame_id = "map"
			map_msg.header.stamp    = rospy.Time.now()

			map_msg.info= MapMetaData()
			map_msg.info.map_load_time = rospy.Time.now()
			map_msg.info.height = self.HEIGHT      #Unit: Pixel
			map_msg.info.width  = self.WIDTH      #Unit: Pixel
			map_msg.info.resolution = self.RESOLUTION
			
			#Set new stacked occupancy grid to center at robot
			map_msg.info.origin = Pose()
			map_msg.info.origin.position = Point()
			map_msg.info.origin.position.x = self.point.x - self.WIDTH*self.RESOLUTION/2      #Unit: Meter
			map_msg.info.origin.position.y = self.point.y - self.HEIGHT*self.RESOLUTION/2      #Unit: Meter
			map_msg.info.origin.position.z = 0

			#Set orientation with respect to map
			#Currently testing with costmap generated from /scan
			#Orientation is set to be that of local_costmap (0,0,0,1)
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
