#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

ONE_MPH = 0.44704

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.INFO)

        self.dbw_enabled = False # drive-by-wire status
        self.current_velocity = None
        self.twist_cmd = None

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        throttle_control_kp = rospy.get_param('~throttle_control_kp', 0.1)
        throttle_control_kd = rospy.get_param('~throttle_control_kd', 1.0)
        throttle_control_ki = rospy.get_param('~throttle_control_ki', 0.0)

        min_speed = 1. * ONE_MPH

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Create `TwistController` object
        self.controller = Controller(throttle_control_kp=throttle_control_kp,
                                     throttle_control_kd=throttle_control_kd,
                                     throttle_control_ki=throttle_control_ki,
                                     min_speed=min_speed,
                                     vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     brake_deadband=brake_deadband,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     wheel_radius=wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_status_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

        print("initialised DBW node")
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Upstream or downstream not intialised yet
            if (self.twist_cmd is None) or (self.current_velocity is None):
                continue

            # Compute throttle, brake and steering angle and send it to car
            throttle, brake, steering = self.controller.control(self.twist_cmd, self.current_velocity, self.dbw_enabled)
            self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_status_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.logwarn("[dbw_node] drive-by-wire: %s", self.dbw_enabled)
        return

    def twist_cb(self, msg):
        self.twist_cmd = msg
        #rospy.loginfo("twist_cb %s\n", self.twist_cmd)
        return

    def current_velocity_cb(self, msg):
        self.current_velocity = msg
        #rospy.loginfo("current_velocity_cb %s\n", self.current_velocity)
        return

if __name__ == '__main__':
    DBWNode()
