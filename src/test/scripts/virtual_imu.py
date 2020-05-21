#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class odom:
    def __init__(self):
        self.orientation = Quaternion(0,0,0,1)
        self.stamp = 'a'
        self.covariance = [None]*36

    def callback(self,data):
        self.orientation = data.orientation
        self.stamp = data.header.stamp
        self.covariance = data.orientation_covariance

    def run(self):
        rospy.init_node("virtual_odom")
        pub = rospy.Publisher("virtual/imu",Imu,queue_size=1)
        sub = rospy.Subscriber("imu",Imu,self.callback) 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Imu()
            msg.orientation = self.orientation
            msg.orientation_covariance = self.covariance
            msg.header.stamp = self.stamp
            msg.header.frame_id = "odom"

            pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        main = odom()
        main.run()
    except rospy.ROSInterruptException:
        pass
