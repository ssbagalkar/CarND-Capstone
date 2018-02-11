from yaw_controller import YawController
from pid import PID
import rospy

GAS_DENSITY = 2.858

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.throttle_controller = PID(kwargs['throttle_control_kp'],
                                       kwargs['throttle_control_ki'],
                                       kwargs['throttle_control_kd'])
        self.steer_controller = YawController(kwargs['wheel_base'],
                                              kwargs['steer_ratio'],
                                              kwargs['min_speed'],
                                              kwargs['max_lat_accel'],
                                              kwargs['max_steer_angle'])

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.previous_time = None
        return

    def control(self, target_twist, current_twist, dbw_enabled):

        throttle = 0.
        brake = 0.
        steer = 0.

        current_time = rospy.get_time()

        if (self.previous_time is None) or (dbw_enabled is False):
            self.throttle_controller.reset()
            self.previous_time = current_time

        else:
            sample_time = current_time - self.previous_time
            self.previous_time = current_time

            # compute acceleration control
            speed_error = target_twist.twist.linear.x - current_twist.twist.linear.x
            throttle = self.throttle_controller.step(speed_error, sample_time)

            # stay within accel_limit and decel_limit
            if throttle > self.accel_limit:
                throttle = self.accel_limit
            if throttle < self.decel_limit:
                throttle = self.decel_limit

            # compute brake torque : wheel_radius * (vehicle_mass + fuel_mass) * acceleration
            if throttle < 0:
                brake = -throttle * self.wheel_radius * (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)
                throttle = 0.
            elif throttle < self.brake_deadband:
                brake = 0.
                throttle = 0.
            else:
                brake = 0.

            # compute steering angle
            steer = self.steer_controller.get_steering(target_twist.twist.linear.x,
                                                       target_twist.twist.angular.z,
                                                       current_twist.twist.linear.x)

            rospy.logdebug("sample_time: %f, speed_error: %f, throttle: %f, brake: %f, steer: %f",
                           sample_time, speed_error, throttle, brake, steer)

        # TODO: Do we need to low pass filter throttle, brake or steering?
        # TODO: Do we need to low pass filter velocity input into throttle and steer control?

        return throttle, brake, steer