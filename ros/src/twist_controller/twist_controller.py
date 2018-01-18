from yaw_controller import YawController
from pid import PID

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

        return

    def control(self, target_twist, current_twist, dbw_enabled):
        throttle = 1. # todo
        brake = 0.      # todo
        steer = self.steer_controller.get_steering(target_twist.twist.linear.x,
                                                   target_twist.twist.angular.z,
                                                   current_twist.twist.linear.x)
        return throttle, brake, steer