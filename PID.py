#from time import time

class PIDController:
    def __init__(self):
        self.min_out_limit, self.max_out_limit = 0.0, 1.0
        self.kp, self.ki, self.kd = 0.025, 0.001, 0.1
        self.proportional_term, self.integral_term, self.derivative_term = 0.0, 0.0, 0.0
        self.last_value, self.last_time = 0.0, 0.0
        self.time_sample = 0.025

    def limit_value(self, value):
        if value > self.max_out_limit:
            return self.max_out_limit
        else:
            return max(value, self.min_out_limit)

    def calc_pid(self, current_value, limit_value, now):
        #now = float(time())
        change_in_time = float(now - self.last_time)

        if change_in_time >= self.time_sample:
            error = limit_value - current_value
            change_in_values = current_value - self.last_value

            self.proportional_term = self.kp * error
            self.integral_term = self.limit_value(
                self.integral_term + self.ki * error
            )
            self.derivative_term = self.kd * -change_in_values

            self.last_value = current_value
            self.last_time = now

        return self.limit_value(self.proportional_term + self.integral_term * self.ki + self.derivative_term)

    def limit_output(self, min, max):
        if min > max:
            return
        self.min_out_limit = min
        self.max_out_limit = max
        self.integral_term = self.limit_value(self.integral_term)

    def adjust_pid(self, kp=None, ki=None, kd=None):
        if kp is None:
            return self.kp
        if ki is None:
            return self.ki
        if kd is None:
            return self.kd
        if kp >= 0: self.kp = kp 
        if ki >= 0: self.ki = ki
        if kd >= 0: self.kd = kd