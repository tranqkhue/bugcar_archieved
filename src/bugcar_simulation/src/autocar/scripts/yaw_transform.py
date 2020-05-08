#!/usr/bin/env python


import rospy
import tf
import numpy as np
# ROS messages.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class QuatToEuler():
    def __init__(self):
        self.got_new_msg_imu = False
        self.got_new_msg_odom = False
        self.euler_msg_imu = Odometry()
        self.euler_msg_odom= Odometry()
        
        # Create subscribers and publishers.
        #sub_imu   = rospy.Subscriber("imu", Imu, self.imu_callback)
        sub_imu   = rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback)
        sub_odom  = rospy.Subscriber("local_velocity", Odometry, self.odom_callback)
        pub_euler_odom = rospy.Publisher("euler_odom", Odometry, queue_size=10)
        pub_euler_imu = rospy.Publisher("euler_imu", Odometry, queue_size=10)
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg_imu:
                pub_euler_imu.publish(self.euler_msg_imu)
                self.got_new_msg_imu = False
            if self.got_new_msg_odom:
                pub_euler_odom.publish(self.euler_msg_odom)
                self.got_new_msg_odom = False

    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.fill_euler_msg_odom(msg, r, p, y)

    # IMU callback function.
    def imu_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.fill_euler_msg_imu(msg, r, p, y)

    # Fill in Euler angle message.
    def fill_euler_msg_imu(self, msg, r, p, y):
        self.got_new_msg_imu = True
        self.euler_msg_imu.header.stamp = msg.header.stamp
        self.euler_msg_imu.pose.pose.orientation.x  = np.degrees(r)
        self.euler_msg_imu.pose.pose.orientation.y = np.degrees(p)
        self.euler_msg_imu.pose.pose.orientation.z   = np.degrees(y)

    def fill_euler_msg_odom(self, msg, r, p, y):
        self.got_new_msg_odom = True
        self.euler_msg_odom.header.stamp = msg.header.stamp
        self.euler_msg_odom.pose.pose.orientation.x  = np.degrees(r)
        self.euler_msg_odom.pose.pose.orientation.y = np.degrees(p)
        self.euler_msg_odom.pose.pose.orientation.z   = np.degrees(y)

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass