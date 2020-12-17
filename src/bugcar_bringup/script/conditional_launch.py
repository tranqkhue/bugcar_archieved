#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import roslaunch
import tf
import nump as np

from geometry_msgs.msg import Twist
from bno055_usd_stick_msgs.msg import Output as BNO055_OUTPUT
from bno055_usd_stick_msgs.msg import CalibrationStatus as BNO055_STATUS
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg    import Odometry

class CustomLaunch():
    def __init__(self):
        self.main_folder         = "/home/quan/bugcar/src/bugcar_bringup/"
        self.localization_launch = self.main_folder + "launch/bugcar_robot_localization.launch"
        self.gps_launch          = self.main_folder + "launch/single_gps.launch"
        self.motor_driver_launch = self.main_folder + "launch/roboclaw.launch"
        self.imu_launch          = self.main_folder + "launch/bno055.launch"
        self.rc_launch           = self.main_folder + "launch/rc_controller.launch"

        self.gps_stat_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_status_callback, queue_size=1)
        self.imu_stat_sub = rospy.Subscriber('/bno055/output', BNO055_OUTPUT, self.imu_status_callback, queue_size=1)
        self.raw_odom_sub = rospy.Subscriber('/odom/wheel', Odometry, (lambda data: self.odom_data = data), queue_size=1)
        self.raw_imu_sub  = rospy.Subscriber('/imu/data', Imu, (lambda data: self.imu_data = data), queue_size=1) 
        self.vel_test_pub = rospy.Publisher('test_vel', Twist, queue_size=1)

        self.imu_status = BNO055_STATUS()
        self.gps_status = NavSatFix()
        self.imu_data   = Imu()
        self.odom_data  = Odometry()

        self.test_vel   = Twist()
        self.max_test_vel_value = 1    # rad/s
        self.test_vel_step      = 0.1  # rad/s
        self.test_vel_tolerance = 0.01 # relative tolerance

        self.gps_Hacc = float();
        self.gps_Vacc = float();

        self.check_list = {'gps' : False,\
                           'imu' : False,\
                           'odom': False}

    def imu_status_callback(self,data):
        self.imu_status = data
        if self.imu_status.calibration_status.system == 3:
            self.check_list['imu'] = True
    
    def gps_status_callback(self,data):
        gps_covariance = data.position_covariance
        self.gps_Hacc = sqrt(gps_covariance[0])
        self.gps_Vacc = sqrt(gps_covariance[8])
        if self.gps_Hacc < 10.0 and self.gps_Vacc < 10.0:
            self.check_list['gps'] = True

    def vel_test_odom(self): 
        wait_duration = rospy.Duration(2, 0)
        self.test_vel.angular.z = 0.0

        for i in range(self.max_test_vel_value / self.test_vel_step):
            # Make sure that the imu is not disconnected and reinitialize during run
            if self.imu_status.calibration_status.system != 3:
                rospy.logerr('Imu is not calibrated properly')
                rospy.logerr('Please check for stable power source connection and re-calibrate')
                return False

            self.test_vel.angular.z += self.test_vel_step
            self.vel_test_pub.publish(self.test_vel)
            rospy.sleep(0.1)
            rospy.sleep(wait_duration)
            error = np.allclose(tf.euler_from_quaternion(self.imu_data.orientation),\
                                tf.euler_from_quaternion(self.odom_data.pose.pose.orientatioon),\
                                rtol = self.test_vel_tolerance)
            if error:
                rospy.logwarn('At ' + str(self.test_vel.angular.z) + ': \nRelative error between odometry and imu is greater than ' + str(self.test_vel_tolerance))
                rospy.logwarn('     Imu orientation report: ' + str(tf.euler_from_quaternion(self.imu_data.orientation)))
                rospy.logwarn('Odometry orientation report: ' + str(tf.euler_from_quaternion(self.imu_data.orientation)))
        
        rospy.loginfo('Continue? (Y/N) ')
        cont = input()
        if cont == "N" or "n":
            return False
        
        self.check_list['odom'] = True
        return True
            
    
    def check_sequence(self):
        '''
            Check for status of sensors:
            - GPS covariance -> turn on robot_localization and osm_planner with updated datum
            - IMU calibration status
            - Odometry vs Imu data (check for yaw and angular velocity)
        '''
        rospy.loginfo('Please calibrate the imu')
        while !self.check_list['imu']:
            pass
        rospy.loginfo('Imu has been successfully calibrated')

        rospy.loginfo('Starting odometry test')
        if self.vel_test_odom():
            pass
        else:
            rospy.signal_shutdown("Terminate at angular velocity check")
        rospy.loginfo('Motor Driver is configured properly')

        rospy.loginfo('Waiting for GPS positioning convergence')
        while !self.check_list['gps']:
            pass
        rospy.loginfo('GPS positioning has converged')

        rospy.loginfo('============================')
        rospy.loginfo()
        

    def main(self):
        rospy.init_node('custom_launch')
        self.check_sequence()




rospy.init_node('conditional_launch')

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/quan/bugcar/src/bugcar_bringup/launch/osm_move_base.launch"])
launch.start()