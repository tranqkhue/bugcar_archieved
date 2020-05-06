#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf


velocity_x = 0.00
velocity_y = 0.00
heading = 0.00

class gps_heading_velocity:
    def __init__(self):
        self.map_velocity = 0
        self.velocity_x   = 0
        self.covariance_x = 0
        self.covariance_y = 0
        self.covariance_velocity   = map(list,np.zeros((1,36),dtype=float))[0]
        self.covariance_heading    = map(list,np.zeros((1,36),dtype=float))[0]
        self.velocity_y   = 0
        self.heading      = 0
        self.min_heading_velocity = 0.4
        self.stamp=rospy.Time()
    #--------------------------------------------------------------------------------
    def callback(self, data):
        #the linear.x and linear.y is west-east and south-north map coordinate system
        self.velocity_x = data.twist.twist.linear.x
        self.velocity_y = data.twist.twist.linear.y
        self.map_velocity = np.sqrt(self.velocity_x**2+self.velocity_y**2)
        self.stamp=data.header.stamp

        # get velocity_x and velocity_y covariance
        self.covariance_x = data.twist.covariance[0]
        self.covariance_y = data.twist.covariance[7]
        self.covariance_velocity[0] = float(np.sqrt(self.covariance_x**2+self.covariance_y**2))

        # calculate heading from map velocity 
        self.heading = np.arctan2(self.velocity_y,self.velocity_x)
        self.covariance_heading[35] = np.arctan2(self.covariance_velocity[0], self.map_velocity)
    #--------------------------------------------------------------------------------
    #def calculate_heading(self):
        
    #--------------------------------------------------------------------------------    
    def run(self):
        rospy.init_node("fix_velocity")
        pub = rospy.Publisher("/local_velocity",Odometry,queue_size = 1)
        fix_velocity_sub = rospy.Subscriber("ublox/fix_velocity", TwistWithCovarianceStamped, self.callback) 
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Odometry()
            msg.header.stamp = self.stamp #rospy.Time.now()
            msg.header.frame_id = 'base_link'
            quat = tf.transformations.quaternion_from_euler(0, 0, self.heading)
            msg.pose.pose.orientation.z = quat[2]
            msg.pose.pose.orientation.w = quat[3]
            msg.pose.covariance = tuple(self.covariance_heading)
            
            msg.twist.twist.linear.x = self.map_velocity
            msg.twist.covariance = tuple(self.covariance_velocity)
            pub.publish(msg)
            rate.sleep()
            

if __name__ == "__main__":
    try:
        main = gps_heading_velocity()
        main.run()
    except rospy.ROSInterruptException:
        pass