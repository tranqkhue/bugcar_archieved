#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import ToLL

class OdomToLL():
    def __init__(self):
        self.odom=Odometry()
        self.LLValue=NavSatFix()
        self.received_new_value=False

        odom_lis=rospy.Subscriber("odom/input", Odometry, self.odom_callback)
        LL_pub=rospy.Publisher("fix/output", NavSatFix, queue_size=10)

        self.ToLLSrv=rospy.ServiceProxy('toLL', ToLL)
        while not rospy.is_shutdown():
            if self.received_new_value:
                LL_pub.publish(self.LLValue)
                self.received_new_value=False
        

    def odom_callback(self, msg):
        mappoint=Point()
        mappoint.x=msg.pose.pose.position.x
        mappoint.y=msg.pose.pose.position.y
        mappoint.z=0

        geopoint=self.ToLLSrv(mappoint)
        self.LLValue.header.frame_id='base_link'
        self.LLValue.header.stamp=msg.header.stamp
        self.LLValue.header.seq=msg.header.seq
        self.LLValue.latitude=geopoint.ll_point.latitude
        self.LLValue.longitude=geopoint.ll_point.longitude
        self.received_new_value=True

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('PointToLL')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        PointToLL = OdomToLL()
    except rospy.ROSInterruptException: pass

        