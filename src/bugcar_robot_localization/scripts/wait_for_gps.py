#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import numpy
from geographic_msgs.msg import GeoPose
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum

gps_status = False
lat_lon = GeoPose()

lat_lon.orientation.x = 0
lat_lon.orientation.y = 0
lat_lon.orientation.z = 0
lat_lon.orientation.w = 1
lat_lon.position.altitude = 0

def gps_status_callback(data):
    global gps_status
    global lat_lon
    gps_covariance = data.position_covariance
    gps_Hacc = numpy.sqrt(gps_covariance[0])
    gps_Vacc = numpy.sqrt(gps_covariance[8])
    if gps_Hacc < 10.0 and gps_Vacc < 10.0:
        gps_status = True
        rospy.loginfo(str(data.latitude) + " " + str(data.longitude))
        lat_lon.position.latitude   = data.latitude
        lat_lon.position.longitude = data.longitude

if __name__ == "__main__":
    rospy.init_node('wait_for_gps')
    gps_stat_sub = rospy.Subscriber('/gps/fix', NavSatFix, gps_status_callback, queue_size=1)
    rospy.wait_for_service('/datum')
    rospy.loginfo("yo")
    try:
        while not gps_status:
            pass
        set_datum = rospy.ServiceProxy('/datum', SetDatum)
        resp = set_datum(lat_lon)
        rospy.set_param('/navsat_transform/datum',[lat_lon.position.latitude,\
                                                   lat_lon.position.longitude,\
                                                   0.])
	rospy.loginfo("done")
    except rospy.ServiceException as e:
	print("Service call failed: %s" %e)
    
    rospy.spin()
    rospy.signal_shutdown("Done setting datum for robot_localization")




