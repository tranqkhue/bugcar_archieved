import rospy
from sensor_msgs.msg import NavSatFix
from bugcar_google_map_global_planner.srv import GetPathLL

ll_position = NavSatFix()

rospy.init_node('service_client')
get_path = rospy.ServiceProxy('get_path_ll', GetPathLL)
response = get_path(ll_position)
path     = response.goal_path
print(path)