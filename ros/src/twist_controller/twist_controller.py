from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, max_angular_velocity, accel_limit, decel_limit):
        self.max_angular_velocity = max_angular_velocity
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

        steer_kp = 1.
        steer_ki = 0.
        steer_kd = 0.

        self.steer_pid = PID(steer_kp,
                             steer_ki,
                             steer_kd,
                             0. - max_angular_velocity,
                             max_angular_velocity)

        throttle_kp = 1.
        throttle_ki = 0.
        throttle_kd = 0.

        self.throttle_pid = PID(throttle_kp,
                                throttle_ki,
                                throttle_kd,
                                decel_limit,
                                accel_limit)

    def control(self, goal_acceleration, goal_angular_velocity,
                current_velocity, deltat, dbw_enabled):
        acceleration = self.throttle_pid.step(goal_acceleration, deltat)
        angular_velocity = self.steer_pid.step(goal_angular_velocity, deltat)

        if not dbw_enabled:
            self.throttle_pid.reset()
            self.steer_pid.reset()

        return acceleration, angular_velocity

    def update_steer_pid(self, p, i, d):
        self.steer_pid.update_gains(p, i, d)

    def update_throttle_pid(self, p, i, d):
        self.throttle_pid.update_gains(p, i, d)

    def reset_throttle_pid(self):
        self.throttle_pid.reset()


class GainController(object):
    def __init__(self, max_throttle, max_brake, max_steer_angle, delay_seconds, steer_ratio):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steer_angle = max_steer_angle
        self.delay_seconds = delay_seconds
        self.steer_ratio = steer_ratio

    def control(self, goal_acceleration, goal_angular_velocity,
                      linear_speed, angular_velocity,
                      linear_acceleration, angular_acceleration,
                      deltat, dbw_enabled):
        throttle = goal_acceleration
        steer_angle = goal_angular_velocity * self.steer_ratio
        brake = 0
        if throttle < 0.:
            brake = 0. - throttle
            throttle = 0.
        return throttle, brake, steer_angle
