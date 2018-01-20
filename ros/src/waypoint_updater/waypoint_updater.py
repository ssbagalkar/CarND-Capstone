#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Define all topics to which we will subscribe:
        # 1. current_pose
        # 2.base_waypoints
        # 3.traffic_waypoint
        # 4.obstacle_waypoint

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # Initiate flags
        self.current_position = None
        self.waypoints = None
        self.max_velocity = None
        self.frame_id = None

        rospy.spin()

    def pose_cb(self, msg):
        # return the current position of the car
        self.current_position = msg.pose
        self.frame_id = msg.header.frame_id
        self.publish_waypoints()


    def waypoints_cb(self, waypoints):
        #return the x,y of waypoints
        self.waypoints = waypoints.waypoints


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    # This function was given in path planning project.Implementing the same in python
    def get_closest_waypoint(self, pose, waypoints):
        closest_length = float('inf')
        closest_waypoint = 0
        for i in range(len(waypoints)):
            distance = self.distance(pose.pose.position, waypoints.pose.pose.position)
            if distance < closest_length:
                closest_length = distance
                closest_waypoint = i

        return closest_waypoint

    # This function was given in path planning project.Implementing the same in python
    def next_waypoint(self, pose, waypoints):
        closest_waypoint = self.get_closest_waypoint(pose, waypoints)
        next_x = waypoints[closest_waypoint].pose.pose.position.x
        next_y = waypoints[closest_waypoint].pose.pose.position.y

        heading = math.atan2((next_y - pose.position.y), (next_x - pose.position.x))
        x_quaternion = pose.orientation.x
        y_quaternion = pose.orientation.y
        z_quaternion = pose.orientation.z
        w_quaternion = pose.orientation.w
        _,_,get_euler_angles= tf.transformations.euler_from_quaternion([x_quaternion,y_quaternion,z_quaternion,w_quaternion])
        angle = abs(get_euler_angles[-1] - heading)

        if angle > (math.pi / 4):
            closest_waypoint += 1

        return closest_waypoint

    # Define a lane object which will be used to publish final waypoints ahead of the car
    def lane_object(self,frame_id, waypoints):
        lane = Lane()
        lane.header.frame_id = frame_id
        lane.waypoints = waypoints
        lane.header.stamp = rospy.Time.now()

        return lane

    def publish_waypoints(self):
        # Get the index of next waypoint
        index_of_next_waypoint = self.next_waypoint(self.current_position,self.waypoints)

        # Define the starting and end index of waypoints
        start_waypoint = index_of_next_waypoint
        end_waypoint = index_of_next_waypoint+LOOKAHEAD_WPS

        # Lookahead waypoints
        lookahead_waypoints = self.waypoints[start_waypoint:end_waypoint]


        lane = lane_object(self.frame_id,lookahead_waypoints)

        self.final_waypoints_pub.publish(lane)
        
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
