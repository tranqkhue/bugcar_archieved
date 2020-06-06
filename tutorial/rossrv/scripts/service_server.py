import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path

from bugcar_google_map_global_planner.srv import GetPathLL
from bugcar_google_map_global_planner.srv import GetPathMap

rospy.init_node("service_server")

def callback(request):
	goal = request.latlong_goal
	print(goal)
	response = Path()
	return(response)

service_ll  = rospy.Service('get_path_ll', GetPathLL, callback)
rospy.spin() 
