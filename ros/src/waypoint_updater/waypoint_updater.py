#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):    
    def __init__(self):
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_z = 0.0
        self.traffic_signal = -1
        
        rospy.init_node('waypoint_updater')
        rospy.loginfo('waypoint_log')
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        self.wpt_list = []
        self.car_orientation = []
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        while not rospy.is_shutdown():
            car_pts = [self.car_x,self.car_y,self.car_z]
            waypoints = self.wpt_list
            orientation_list = self.car_orientation
            try:
                (car_roll, car_pitch, car_yaw) = euler_from_quaternion(orientation_list)
                next_wpt = self.next_waypoint(car_pts,car_yaw,waypoints)
                next_x,next_y,next_z,next_val_x = self.find_next_pts(waypoints,next_wpt)
                self.publish_final_wpt(next_x,next_y,next_z,next_val_x)
            except:
                pass
            rospy.Rate(10).sleep()
            
    
    def traffic_cb(self,msg):
        self.traffic_signal = msg.data

    def pose_cb(self, msg):
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        self.car_z = msg.pose.position.z
        self.car_orientation=[msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
   

    def next_waypoint(self,car_pts,car_yaw,waypoints):

        #calculate the nearest waypoint number 
        closest = 1000
        closest_wpt = 0
        for i in range(len(waypoints)):
            wpt = [waypoints[i].pose.pose.position.x, waypoints[i].pose.pose.position.y, waypoints[i].pose.pose.position.z]
            dist = self.dist_btw_pts(wpt,car_pts)
            if dist < closest:
                closest = dist
                closest_wpt = i 
        
        #check the angle between wpt and car pt
        wpt_x = waypoints[closest_wpt].pose.pose.position.x
        wpt_y = waypoints[closest_wpt].pose.pose.position.y

        #calculate the direction angle between the cat pts and the waypoint
        direction = math.atan2((wpt_y-car_pts[1]),(wpt_x-car_pts[0]))

        # calculate the difference between the car yaw and heading direction 
        angle = math.fabs(car_yaw-direction)

        # if it is greater than 45 degree, use the next wpts 
        if angle > math.pi/4.0:
            closest_wpt +=1
        return closest_wpt
		
    def find_next_pts(self,waypoints,next_wpt):
        next_x =[]
        next_y =[]
        next_z =[]
        next_val_x = []

        if self.traffic_signal != -1:
            # do the implimentation to slow down the car on TL
            final_wpt = self.traffic_signal
            for i in range(next_wpt, final_wpt):
                index = i % len(waypoints)
                next_x.append(waypoints[index].pose.pose.position.x)
                next_y.append(waypoints[index].pose.pose.position.y)
                next_z.append(waypoints[index].pose.pose.position.z)
                next_val_x.append(0.0)

        else :
            final_wpt = next_wpt + LOOKAHEAD_WPS
            for i in range(next_wpt,final_wpt):
                index = i%len(waypoints)
                next_x.append(waypoints[index].pose.pose.position.x)
                next_y.append(waypoints[index].pose.pose.position.y)
                next_z.append(waypoints[index].pose.pose.position.z)
                next_val_x.append(waypoints[index].twist.twist.linear.x)

        return next_x,next_y,next_z ,next_val_x

    def publish_final_wpt(self,next_x,next_y,next_z,next_val_x):
        wpts = []
        p = Waypoint()
        for i in range(len(next_x)):
            p.pose.pose.position.x = next_x[i]
            p.pose.pose.position.y = next_y[i]
            p.pose.pose.position.z = next_z[i]
            
            p.twist.twist.linear.x = next_val_x[i]
            wpts.append(p)
        lane = Lane()
        lane.header.frame_id = "/world"
        lane.header.stamp = rospy.Time(0)
        lane.waypoints =  wpts
        self.final_waypoints_pub.publish(lane)       

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.wpt_list = waypoints.waypoints

    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, a, b):
        x = a.x - b.x
        y = a.y - b.y
        z = a.z - b.z
        return math.sqrt(x*x + y*y + z*z)

    def dist_btw_pts (self,pt1,pt2):
        return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2  + (pt1[2]-pt2[2])**2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')