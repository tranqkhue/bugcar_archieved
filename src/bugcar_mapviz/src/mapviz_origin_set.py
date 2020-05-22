#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
rospy.init_node('mapviz_origin_set')

while not rospy.is_shutdown():
	try:
		[lat,long,altitude] = rospy.get_param('/navsat_transform/datum')
		break
	except KeyError:
		rospy.logerr('Cannot get origin from /navsat_transform/datum')

param = {'latitude': lat,
	     'longitude': long,
	     'altitude': altitude,
	     'name': 'swri',
	     'heading': 0.0} #Heading NED

param = [param]
rospy.set_param('/initialize_origin/local_xy_origins', param)
