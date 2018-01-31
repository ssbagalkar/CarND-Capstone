#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb) #may be helpful in simulator, but not available when running on Carla
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def distance(self, a_points, b_points):
        x = a_points[0] - b_points[0]
        y = a_points[1] - b_points[1]

        z = 0
        if len(a_points) == 3 and len(b_points) == 3:
            z = a_points[2] - b_points[2]
        return math.sqrt(x*x + y*y + z*z)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_waypoint = 0
        closest_distance = float('inf')
        # defensive check to make sure self.waypoints have been loaded
        if self.waypoints:
            for i in range(len(self.waypoints)):
                wp_pos = self.waypoints[i].pose.pose.position
                distance = self.distance([pose.position.x, pose.position.y, pose.position.z], [wp_pos.x,wp_pos.y,wp_pos.z])
                if distance < closest_distance:
                    closest_distance = distance
                    closest_waypoint = i
        return closest_waypoint

    #TODO: refactor this #DRY
    def get_closest_stop_line(self, pose, stop_line_positions):
        closest_stop_line = None
        closest_distance = float('inf')
        for i in range(len(stop_line_positions)):
            distance_2D = self.distance([pose.position.x, pose.position.y], stop_line_positions[i])
            if distance_2D < closest_distance and distance_2D < 80:
                closest_distance = distance_2D
                closest_stop_line = i
        return closest_stop_line

    def get_light_wp(self, stop_line_position):
        closest_waypoint = 0
        closest_distance = float('inf')
        if self.waypoints:
            for i in range(len(self.waypoints)):
                wp_pos = self.waypoints[i].pose.pose.position
                distance = self.distance(stop_line_position, [wp_pos.x, wp_pos.y])
                if distance < closest_distance:
                    closest_distance = distance
                    closest_waypoint = i
        return closest_waypoint
        

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

            sl_pos_index = self.get_closest_stop_line(self.pose.pose, stop_line_positions)

            if sl_pos_index != None:
                if(len(self.lights) > 0):
                    light = self.lights[sl_pos_index]
                else:
                    light = True
                light_wp = self.get_light_wp(stop_line_positions[sl_pos_index])

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
