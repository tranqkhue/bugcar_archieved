#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#============================================================================================
'''  Import libraries  '''
import requests
import polyline
import utm

import rospy
import tf as ros_tf

from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from bugcar_google_map_global_planner.srv import GetPathLL
from bugcar_google_map_global_planner.srv import GetPathMap

import numpy as np
#============================================================================================



#============================================================================================
'''   Init ROS node, input and output topics   '''
rospy.init_node("Google_Map_Get_Path_Service")
tf_listener = ros_tf.TransformListener()    

gps_position_input  = "gps/filtered"
#============================================================================================



#============================================================================================
'''   This function converts a point in map coordinate into lat_long   '''
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
#============================================================================================  



#============================================================================================
'''   This function gets the GPS coordinate and convert it into a dict   '''
def gps_coordinate_callback(msg):
    #msg type: sensor_msgs/NavSatFix
    global gps_position
    gps_position = {'lat': msg.latitude, 'long': msg.longitude}
  
#============================================================================================

  
    
#============================================================================================
'''   THIS WHOLE SECTION IS FOR PATH CALCULATION AND PATH_MSGS GENERATION!!!   '''
#--------------------------------------------------------------------------------------------
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
    
#--------------------------------------------------------------------------------------------
    
#--------------------------------------------------------------------------------------------
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
    
    #DANGER ZONE!!!
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
    
#--------------------------------------------------------------------------------------------
'''   END OF PATH CALCULATION AND PATH_MSGS GENERATION!!!   '''
#============================================================================================ 
    
   
 
#============================================================================================
'''   This function gets the goal as GPS coordinate 
      I the vehicle is busy following a plan: log an error
      If not: calculate a path, globalize it so the Path publisher can catch it  
'''
def ll_goal_callback(request):
    
    #For GetPathLL srv
    #Input msg type: sensor_msgs/NavSatFix    latlong_goal
    #Output type:    nav_msgs/Path            goal_path
    
    rospy.loginfo('New latlong-coordinate goal received!')
    goal_lat  = request.latlong_goal.latitude
    goal_long = request.latlong_goal.longitude
    geo_goal = {'lat': goal_lat, 'long': goal_long}
    output_map_path_msg = calculate_path(gps_position, geo_goal)
    return(output_map_path_msg)

#============================================================================================



#============================================================================================
def map_goal_callback(request):
    
    #FRAME_ID OF MAP_GOAL: MAP   
    #For GetPathMap srv
    #Input msg type: move_base_msgs/MoveBaseActionGoal  map_goal
    #Output type:    nav_msgs/Path                      goal_path

    rospy.loginfo('New map-coordinate goal received!')
    goal_x_map = request.map_goal.goal.target_pose.pose.position.x
    goal_y_map = request.map_goal.goal.target_pose.pose.position.y
    geo_goal   = map2ll(goal_x_map, goal_y_map)
    output_map_path_msg = calculate_path(gps_position, geo_goal)
    return(output_map_path_msg)
    
#============================================================================================

    

#============================================================================================
''' __MAIN__ '''
#--------------------------------------------------------------------------------------------        
gps_sub           = rospy.Subscriber(gps_position_input, NavSatFix,           \
                                     gps_coordinate_callback,    queue_size=1)
#--------------------------------------------------------------------------------------------  
ll_goal_service   = rospy.Service('get_path_ll', GetPathLL, ll_goal_callback)
#CAUTION! map_goal_sub receive MoveBaseActionGoal from move_base node! 
#Not from Rviz's PointStamped 
map_goal_service  = rospy.Service('get_path_map', GetPathMap, map_goal_callback)
rospy.spin()