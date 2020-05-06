#!/usr/bin/env python


import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_localization.srv import FromLL
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from gazebo_msgs.msg import ModelStates
import math  

PI=3.14
DEG2RAD=PI/180
RAD2DEG=180/PI

class PID_Calculation:
    def __init__(self, kp, ki ,kd, ki_winding):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.ki_winding=ki_winding
        self.old_error=0
        self.curret_error=0
        self.sum_error=0
        self.PID_result=0
        self.derivative_error=0

    def Integrate_Winding(self):
        if (self.sum_error<-self.ki_winding):
            self.sum_error=-self.ki_winding
        elif (self.sum_error>self.ki_winding):
            self.sum_error=self.ki_winding

    def PID_Calculation(self, current, destination):
        self.curret_error=destination-current
        self.sum_error+=self.curret_error
        self.derivative_error=self.curret_error-self.old_error
        self.old_error=self.curret_error
        self.Integrate_Winding()
        self.PID_result=self.kp*self.curret_error+self.ki*self.sum_error + self.kd*self.derivative_error
        return self.PID_result


class Moving_Robot_Crudely:
    def __init__(self):
        self.vel_command=Twist()

        self.current_x=0
        self.current_y=0
        self.current_theta=0
        self.distance=0

        self.destination=Odometry()
        self.desired_x=0
        self.desired_y=0
        self.desired_theta=0

        
        self.rate=rospy.Rate(19) #Rate 10Hz
        
        #Get some parameter
        desired_lat=rospy.get_param('desired_lat',11.055009)
        desired_long=rospy.get_param('desired_long',106.666215)
        speed=rospy.get_param('crusing_speed',1)

        kp_angular=rospy.get_param('angular_PID/kp',1)
        ki_angular=rospy.get_param('angular_PID/ki',0)
        kd_angular=rospy.get_param('angular_PID/kd',0)
        ki_winding_angular=rospy.get_param('angular_PID/kp_angular',0)

        self.PID_Angle_Calculation= PID_Calculation(kp_angular,ki_angular,kd_angular,ki_winding_angular)
        #Create Publisher and Subcriber
        #self.pose_sub=rospy.Subscriber("odometry/final_value", Odometry, self.pose_callback)
        self.pose_sub=rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_pose_callback) #For Debugging only
        self.cmd_pub=rospy.Publisher("/Diff_Drive/diff_drive_controller/cmd_vel", Twist, queue_size=10)
        self.dest_pub=rospy.Publisher("/destination", Odometry, queue_size=10)
        self.FromLLSrv=rospy.ServiceProxy('fromLL', FromLL)

        #Convert Desired Lat Long to desired x-y map frame
        self.convert_LL_toPoint(desired_lat, desired_long)
        
        self.flag=0
        
        # Main while loop.
        while not rospy.is_shutdown():
            #Calculate the distance and vector
            self.distance, self.desired_theta= self.vector_calulation()
            if (self.distance<0.1):
                self.vel_cmd_Stop()
                rospy.loginfo("Stop")
            elif (abs(self.desired_theta-self.current_theta)<0.01):
                self.vel_cmd_moveForward(speed)
                rospy.loginfo("Moving forward at %d", speed)
            else:
                angular_vel=self.PID_Angle_Calculation.PID_Calculation(self.current_theta, self.desired_theta)
                self.vel_cmd_turntoAngle(angular_vel)
                rospy.loginfo("Turning from angle %f to angle %f at rate %f", self.current_theta*RAD2DEG, self.desired_theta*RAD2DEG, angular_vel*RAD2DEG)
            

            self.draw_destination_line() #For tuning and debugging
            self.cmd_pub.publish(self.vel_command)
            self.rate.sleep()

    def convert_LL_toPoint(self, lat, long):
        geopoint=GeoPoint() #Geography point (lat,long, altitude) being converted
        geopoint.latitude=lat
        geopoint.longitude=long
        geopoint.altitude=0

        desired_point=self.FromLLSrv(geopoint)
        self.desired_x=desired_point.map_point.x
        self.desired_y=desired_point.map_point.y 
    #For Debugging only
    def draw_destination_line(self):
        if (self.flag):
            self.destination.pose.pose.position.x=self.desired_x
            self.destination.pose.pose.position.y=self.desired_y
            self.destination.pose.pose.orientation.z=self.desired_theta*RAD2DEG
            self.destination.pose.pose.orientation.w=self.current_theta*RAD2DEG
            self.dest_pub.publish(self.destination)
            self.flag=0
        else:
            self.destination.pose.pose.position.x=0
            self.destination.pose.pose.position.y=0
            self.destination.pose.pose.orientation.z=self.desired_theta*RAD2DEG
            self.destination.pose.pose.orientation.w=self.current_theta*RAD2DEG
            self.dest_pub.publish(self.destination)
            self.flag=1
    def pose_callback(self, msg):
        self.current_x=msg.pose.pose.position.x
        self.current_y=msg.pose.pose.position.y 
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.current_theta=y

    def gazebo_pose_callback(self, msg):
        self.current_x=msg.pose[1].position.x
        self.current_y=msg.pose[1].position.y 
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w])
        self.current_theta=y
    def vector_calulation(self):
        delta_x = self.desired_x - self.current_x
        delta_y = self.desired_y - self.current_y
        distance=math.sqrt(delta_x**2+delta_y**2)
        angle = math.atan2(delta_y, delta_x)
        return distance, angle

    def vel_cmd_Stop(self):
        self.vel_command.linear.x=0
        self.vel_command.angular.z=0

    def vel_cmd_moveForward(self, speed):
        self.vel_command.linear.x=speed
        self.vel_command.angular.z=0

    def vel_cmd_turntoAngle(self, angle_speed):
        self.vel_command.linear.x=0
        self.vel_command.angular.z=angle_speed

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Moving_Robot_Crudely')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        Moving_Robot = Moving_Robot_Crudely()
    except rospy.ROSInterruptException: pass

