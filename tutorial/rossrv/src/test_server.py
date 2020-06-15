#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#============================================================================================
'''  Import libraries  '''
import requests
import polyline
import utm

import rospy
from nav_msgs.msg import Path

from bugcar_google_map_global_planner.srv import GetPathLL
from bugcar_google_map_global_planner.srv import GetPathMap
from bugcar_google_map_global_planner.srv import GetPathResponse

import numpy as np
#============================================================================================


#============================================================================================
'''   Some GPS and coordinate ultilities   '''

#--------------------------------------------------------------------------------------------
# Get GPS position
def gps_coordinate_callback(msg):
    #msg type: sensor_msgs/NavSatFix
    global gps_position
    gps_position = {'lat': msg.latitude, 'long': msg.longitude}

#--------------------------------------------------------------------------------------------
#Convert position in map frame to lat_long coordinate 
def map2ll(x_map,y_map):
    
    ros_tf_transformer = ros_tf.TransformerROS()
    try:
        tf_listener.waitForTransform("utm", "map", rospy.Time(), \
                                     rospy.Duration(5.0))
    except (ros_tf.LookupException):
        rospy.logerr('Cannot find utm-map tf')
        return None  
    (trans, rot) = tf_listener.lookupTransform('utm', 'map', rospy.Time(0))

    map_utm_tf_matrix = ros_tf_transformer.fromTranslationRotation(tuple(trans), tuple(rot))
    map_point = [x_map, y_map, 0, 1]
    utm_point = np.dot(map_utm_tf_matrix, map_point)
    
    UTM_ZONE_NUMBER = 48
    UTM_ZONE_LETTER = 'P'
    
    ll_point = utm.to_latlon(utm_point[0], utm_point[1], UTM_ZONE_NUMBER, UTM_ZONE_LETTER)
    ll_point = {'lat':ll_point[0], 'long':ll_point[1]}
    return(ll_point)

#--------------------------------------------------------------------------------------------
#Convert UTM coordinates to map frame
def utm_map_tf(utm_points):
    
    ros_tf_transformer = ros_tf.TransformerROS()
    try:
        tf_listener.waitForTransform("map", "utm", rospy.Time(), \
                                     rospy.Duration(5.0))
    except (ros_tf.LookupException):
        rospy.logerr('Cannot find utm-map tf')
        return None  
    (trans, rot) = tf_listener.lookupTransform('map', 'utm', rospy.Time(0))
    utm_map_tf_matrix = ros_tf_transformer.fromTranslationRotation(tuple(trans), tuple(rot))
    
    map_points = list()
    for i in utm_points:
        i = i + (0,1) #To fill z and w into the utm_points to become a (4,1) matrix
        map_points.append(list(np.dot(utm_map_tf_matrix, i)))
    
    return(map_points)
#============================================================================================


#============================================================================================
'''   THIS WHOLE SECTION IS FOR PATH CALCULATION AND PATH_MSGS GENERATION!!!   '''

#--------------------------------------------------------------------------------------------
# Calculate heading from (n+1, n) Google Map response points for final path
# As PoseStamped and also Path require each points to have a robot's heading
def calculate_heading(map_points):
    #Calculate heading in map
    #The first heading is based on current heading
    #Then heading is calculated based on the arctan of last and next map points
    
    heading = list()
    for i in range(len(map_points)):
        #------------------------------------------------------------------------------------
        if (i==0):
            try:
                tf_listener.waitForTransform("base_link", "map", rospy.Time(), \
                                             rospy.Duration(5.0))
            except (ros_tf.LookupException):
                rospy.logerr('Cannot find map->base_link tf')
                return None 
            (trans, rot) = tf_listener.lookupTransform('base_link', 'map', rospy.Time(0))
            heading.append(rot)
        #------------------------------------------------------------------------------------
        if (i!=0):
            #np.arctan2: Element-wise arc tangent of x1/x2 choosing the quadrant correctly.
            z_euler_heading = np.arctan2((map_points[i][1]-map_points[i-1][1]), \
                                         (map_points[i][0]-map_points[i-1][0]))
            loop_heading = ros_tf.transformations.quaternion_from_euler(0,0,z_euler_heading)
            heading.append(list(loop_heading))
        #------------------------------------------------------------------------------------
            
    return(heading)  
#--------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------
def generate_path_msg(map_points, map_headings):
    
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'
    
    #----------------------------------------------------------------------------------------
    for i in range(len(map_points)):
        single_point_pose = PoseStamped()
        single_point_pose.header = Header()
        single_point_pose.header.stamp = rospy.Time.now()
        single_point_pose.header.frame_id = "map"
        
        single_point_pose.pose.position.x = map_points[i][0]
        single_point_pose.pose.position.y = map_points[i][1]
        single_point_pose.pose.position.z = 0

        single_point_pose.pose.orientation.x = 0
        single_point_pose.pose.orientation.y = 0
        single_point_pose.pose.orientation.z = map_headings[i][2]
        single_point_pose.pose.orientation.w = map_headings[i][3]
        path_msg.poses.append(single_point_pose)
    #----------------------------------------------------------------------------------------
    
    return(path_msg)    
    
#--------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------
def calculate_path(gps_position, geo_goal):
    #Calculate path in MAP FRAME!
    
    #DANGEROUS ZONE!!!
    #PLEASE CHANGE YOUR OWN API KEY
    #OR NOT, PLEASE DO NOT SPAM THIS API KEY!
    API_KEY = "AIzaSyDc3VNCA56-e-qqel-CfB5rhEr1YwsaXqw"
    #current_pos, goal are dicts{'lat':, 'long':}
    url = "https://maps.googleapis.com/maps/api/directions/json?"
    url = url + "&origin=" + str(gps_position['lat']) + ',' + str(gps_position['long'])
    url = url + "&destination=" + str(geo_goal['lat']) + ',' + str(geo_goal['long'])
    url = url + "&key=" + API_KEY
    response = requests.get(url)

    if (response.ok == False):
        rospy.logerr("404 error to Google Map Direction API server")
        return(None)
    json_response = response.json()
    if (json_response['status'] != "OK"):
        rospy.logerr("Error in response from Google Map Direction API: %s" %json_response['status'])
        return(None)
 
    rospy.loginfo('Done getting navigation information from Google Map!')
    steps = json_response['routes'][0]['legs'][0]['steps']
    polylines = list()
    for i in steps:
        polylines.append(i['polyline']['points'])
    ll_points = list()
    for i in polylines:
        ll_points.extend(polyline.decode(i))
    utm_points = list()
    for i in ll_points:
        utm_points.append((utm.from_latlon(i[0],i[1]))[0:2])
    map_points = utm_map_tf(utm_points)

    #headings index is ordered respectively to map_points index
    map_headings = calculate_heading(map_points)
    map_path_msg = generate_path_msg(map_points, map_headings)
    
    return(map_path_msg)  
    
#============================================================================================
rospy.init_node("Google_Map_Global_Planner")
tf_listener = ros_tf.TransformListener()    

def get_path_ll(ll_request):
    geo_goal = {'lat': ll_request.latlong_goal.latitude, 'long': ll_request.latlong_goal.latitude}
    output_map_path_msg = calculate_path(gps_position, geo_goal)
    srv_output = GetPathResponse()
    GetPathResponse.goal_path = 

def get_path_map(map_request):
	goal_x_map = map_request.map_goal.goal.target_pose.pose.position.x
    goal_y_map = map_request.map_goal.goal.target_pose.pose.position.y
    geo_goal   = map2ll(goal_x_map, goal_y_map)
    output_map_path_msg = calculate_path(gps_position, geo_goal)


#============================================================================================

service_ll  = rospy.Service('get_path_ll', GetPathLL, get_path_ll)
service_map = rospy.Service('get_path_map', GetPathMap, get_path_map)
gps_sub     = rospy.Subscriber(gps_position_input, NavSatFix,         \
                               gps_coordinate_callback,    queue_size=1)
rospy.spin() 