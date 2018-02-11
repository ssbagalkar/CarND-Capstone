#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32

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

    #===================================================================================================================
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_status_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.waypoints = None
        self.num_waypoints = 0
        self.current_pose_msg = None
        self.closest_wp_index = 0
        self.dbw_enabled = False
        self.init_closest_wp_check = True
        self.traffic_signal = -1

        while not rospy.is_shutdown():
            if self.waypoints is None:
                rospy.logwarn("[WaypointUpdater] Waiting for waypoints.")

            elif self.current_pose_msg is None:
                self.init_closest_wp_check = True
                rospy.logwarn("[WaypointUpdater] Waiting for car pose.")

            else:
                self.compute_waypoints()

            rospy.Rate(10).sleep()
        #rospy.spin()

    #===================================================================================================================
    def pose_cb(self, msg):
        self.current_pose_msg = msg
        return

    #===================================================================================================================
    def compute_waypoints(self):
        self.closest_wp_index = self.get_closest_waypoint_index(self.current_pose_msg.pose)
        final_waypoints = self.look_ahead_waypoints(self.closest_wp_index)
        self.final_waypoints_pub.publish(final_waypoints)
        return

    #===================================================================================================================
    def get_closest_waypoint_index(self, current_pose):
        # logic: start search from index set last time, as we are going to be somewhere nearby
        nearest_dist = sys.float_info.max
        if self.init_closest_wp_check is True:
            self.init_closest_wp_check = False
            nearest_index = 0
            end_index = nearest_index + self.num_waypoints
            #rospy.logwarn("[waypoint_updater] Reset nearest waypoint search")
        else:
            nearest_index = self.closest_wp_index
            end_index = nearest_index + LOOKAHEAD_WPS

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(nearest_index, end_index):
            waypoint = self.waypoints[i%self.num_waypoints]
            dist = dl(waypoint.pose.pose.position, current_pose.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_index = i
        #rospy.logwarn("nearest_index: %d", nearest_index)
        return nearest_index

    #===================================================================================================================
    def look_ahead_waypoints(self, from_index):

        # TODO update this method to integrate obstacle waypoints and set desired velocity properly

        lane = Lane()
        lane.header.frame_id = self.current_pose_msg.header.frame_id
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = []
        to_index = from_index + LOOKAHEAD_WPS
        stop_car = False
        if self.traffic_signal != -1:
            to_index = self.traffic_signal
            stop_car = True
            rospy.logwarn("[waypoint_updater] STOP LIGHT")

        for i in range(from_index, to_index):
            waypoint = self.waypoints[i%self.num_waypoints]
            if stop_car is True:
                waypoint.twist.twist.linear.x = 0.
            lane.waypoints.append(waypoint)
        return lane

    #===================================================================================================================
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.num_waypoints = len(self.waypoints)
        rospy.logwarn("[WaypointUpdater] Waypoints received.")
        return

    #===================================================================================================================
    def dbw_status_cb(self, msg):
        self.init_closest_wp_check = True
        self.dbw_enabled = msg.data

        #rospy.logwarn("[waypoint_updater] drive-by-wire: %s", self.dbw_enabled)
        return

    #===================================================================================================================
    def traffic_cb(self, msg):
        self.traffic_signal = msg.data
        return

    #===================================================================================================================
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    #===================================================================================================================
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    #===================================================================================================================
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    #===================================================================================================================
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
