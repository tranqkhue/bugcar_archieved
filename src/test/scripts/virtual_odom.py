#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class odom:
    def __init__(self):
        self.orientation = Quaternion(0,0,0,1)
        self.stamp = rospy.Time()
        self.covariance = [None]*36

    def callback(self,data):
        self.orientation = data.orientation
        self.stamp = data.header.stamp
        self.covariance = data.orientation_covariance

    def run(self):
        rospy.init_node("virtual_odom")
        pub = rospy.Publisher("test/odom",Odometry,queue_size=1)
        sub = rospy.Subscriber("imu",Imu,self.callback) 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Odometry()
            msg.pose.pose.orientation = self.orientation
            msg.pose.covariance[0] = self.covariance[0]
            msg.pose.covariance[7] = self.covariance[1]
            msg.pose.covariance[14] = self.covariance[2]
            msg.pose.covariance[21] = self.covariance[3]
            msg.pose.covariance[28] = self.covariance[4]
            msg.pose.covariance[35] = self.covariance[5]
            msg.header.stamp = self.stamp
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"

            pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        main = odom()
        main.run()
    except rospy.ROSInterruptException:
        pass
