#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

class odom:
    def __init__(self):
        self.orientation = Quaternion(0,0,0,1)
        self.angular_velocity = Vector3(0,0,0)
        self.acceleration = Vector3(0,0,0)
        self.stamp = 'a'
        self.covariance = [None]*27

	

    def callback(self,data):
        self.orientation = data.orientation
        self.angular_velocity = data.angular_velocity
        self.acceleration = data.linear_acceleration
        self.stamp = data.header.stamp
        self.covariance[0:9] = data.orientation_covariance
        self.covariance[9:18] = data.angular_velocity_covariance
        self.covariance[18:27] = data.linear_acceleration_covariance

    def run(self):
        rospy.init_node("virtual_odom")
        pub = rospy.Publisher("virtual/imu",Imu,queue_size=1)
        sub = rospy.Subscriber("imu",Imu,self.callback) 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Imu()
            msg.orientation = self.orientation
            msg.orientation_covariance[0] = self.covariance[0]
            msg.orientation_covariance[3] = self.covariance[3]
            msg.orientation_covariance[6] = self.covariance[6]

            msg.angular_velocity_covariance[0] = self.covariance[9]
            msg.angular_velocity_covariance[3] = self.covariance[12]
            msg.angular_velocity_covariance[6] = self.covariance[15]

            msg.linear_acceleration_covariance[0] = self.covariance[18]
            msg.linear_acceleration_covariance[3] = self.covariance[21]
            msg.linear_acceleration_covariance[6] = self.covariance[24]
		
            msg.header.stamp = self.stamp
            msg.header.frame_id = "base_link"

            pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        main = odom()
        main.run()
    except rospy.ROSInterruptException:
        pass
