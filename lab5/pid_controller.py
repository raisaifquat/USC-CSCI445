#!/usr/bin/env python
from utils import clamp


class PIDController:
    def __init__(self, kp, kd, ki, min_output, max_output, min_integral_error, max_integral_error):
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.minOutput = min_output
        self.maxOutput = max_output
        self.min_integral_error = min_integral_error
        self.max_integral_error = max_integral_error

        self.previousError = 0.0
        self.previousTime = None
        self.error_integral = 0

    def update(self, value, target_value, time):
        error = target_value - value
        self.error_integral = clamp(self.error_integral + error, self.min_integral_error, self.max_integral_error)

        p = self.kp * error
        d = 0
        i = 0
        if self.previousTime is not None:
            dt = time - self.previousTime
            if dt > 0:
                d = self.kd * (error - self.previousError) / dt
            i = self.ki * self.error_integral * dt

        output = p + d + i
        self.previousTime = time
        self.previousError = error

        return clamp(output, self.minOutput, self.maxOutput)
