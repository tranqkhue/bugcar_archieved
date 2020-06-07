import rospy
from sensor_msgs.msg import NavSatFix
from bugcar_google_map_global_planner.srv import GetPathMap

ll_position = NavSatFix()
from move_base_msgs.msg import MoveBaseActionGoal

map_goal = MoveBaseActionGoal()

rospy.init_node('service_client')
get_path = rospy.ServiceProxy('get_path_map', GetPathMap)
response = get_path(map_goal)
path     = response.goal_path
print(path)