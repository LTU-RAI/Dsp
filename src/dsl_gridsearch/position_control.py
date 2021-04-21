#!/usr/bin/env python

import rospy, time
#from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from math import pi, sin, cos, atan2, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, Pose, PointStamped
import math

WP_SIZE = 0.5

class Server:
    def __init__(self, current_time=None):
        self.waypoint = False
        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

#        self.vel_publisher = rospy.Publisher('/pixy/reference', Vector3Stamped, queue_size = 10)
        self.vel_publisher = rospy.Publisher('/pixy/reference', Pose, queue_size = 10)

#        self.vel_msg = Vector3Stamped()
        self.vel_msg = Pose()

#        self.waypoint_subscriber = rospy.Subscriber('/dsl_grid3d/optpath', Path, self.waypoint_callback)
        self.waypoint_subscriber = rospy.Subscriber('/dsl_grid3d/path', Path, self.waypoint_callback)
        self.pf_subscriber = rospy.Subscriber('/potential_delta_p', PointStamped, self.pf_callback)
		
    def waypoint_callback(self, msg):
        print("geting waypoints")
        print(msg)
        self.wp_index = 0
        self.path = msg
        self.x_r = msg.poses[self.wp_index].pose.position.x
        self.y_r = msg.poses[self.wp_index].pose.position.y
        self.z_r = msg.poses[self.wp_index].pose.position.z
        self.waypoint = True

    def pf_callback(self, msg):
        print("potential field---------------------------------------------------------------------")
        self.x_pf = msg.point.x
        self.y_pf = msg.point.y
        self.z_pf = msg.point.z
#        print(self.x_pf)

    def read_callback(self, msg):
        self.position_messages = msg
        #print("my postition")
        #print(msg)
        if(self.waypoint):
            self.control()

    def control(self, current_time=None):
        print("controler")
        #if(self.runOnce):
#			self.x_r = x_c
#			self.y_r = y_c
#			self.z_r = z_c
        #  self.runOnce = False

#        print(self.x_pf)    

        x_c = self.position_messages.pose.pose.position.x
        y_c = self.position_messages.pose.pose.position.y
        z_c = self.position_messages.pose.pose.position.z
        q_x = self.position_messages.pose.pose.orientation.x
        q_y = self.position_messages.pose.pose.orientation.y
        q_z = self.position_messages.pose.pose.orientation.z
        q_w = self.position_messages.pose.pose.orientation.w

# at wp?
        if ((x_c - self.x_r) * (x_c - self.x_r)  + (y_c - self.y_r) * (y_c - self.y_r) < WP_SIZE):
            if(self.wp_index < len(self.path.poses) -1 ):
                self.wp_index = self.wp_index + 1
            self.x_r = self.path.poses[self.wp_index].pose.position.x
            self.y_r = self.path.poses[self.wp_index].pose.position.y
            self.z_r = self.path.poses[self.wp_index].pose.position.z


        self.vel_msg.position.x = self.x_r
        self.vel_msg.position.y = self.y_r
        self.vel_msg.position.z = self.z_r

#        self.vel_msg.position.x = self.x_r + self.x_pf
#        self.vel_msg.position.y = self.y_r + self.y_pf
#        self.vel_msg.position.z = self.z_r + self.z_pf
        self.vel_msg.orientation.x = q_x
        self.vel_msg.orientation.y = q_y
        self.vel_msg.orientation.z = q_z
        self.vel_msg.orientation.w = q_w

        print("operatin")
        self.vel_publisher.publish(self.vel_msg)
        return
if __name__ == "__main__":
    try:
        rospy.init_node('position_control', anonymous=True)
        rospy.loginfo("Starting...")
        rospy.Rate(5)	#10Hz

        server = Server()

        while not rospy.is_shutdown():
            rospy.Subscriber("/pixy/truth/NWU", Odometry, server.read_callback)
            rospy.spin()

        rospy.loginfo("Reading finished...")
    except rospy.ROSInterruptException:
        pass
