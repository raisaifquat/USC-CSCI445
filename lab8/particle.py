import numpy as np
import lab8_map
from scipy.stats import norm
import math
from utils import clamp


class Particle:
    def __init__(self, x, y, theta, prev_log_prob, sd_sensor, sd_distance, sd_direction, input_map):
        # coordination
        self.x = x
        self.y = y
        self.theta = theta

        # probability the robot @ this location
        self.prev_log_prob = prev_log_prob

        # constant
        self.sd_sensor = sd_sensor
        self.sd_distance = sd_distance
        self.sd_direction = sd_direction

        # map
        self.map = input_map

    def move(self, turn, distance):
        # if (distance < 0):
        #     raise ValueError('robot cannot move backwards')

        self.theta = self.theta + turn + np.random.normal(0.0, self.sd_direction)
        # self.theta = turn + np.random.normal(0.0, self.sd_direction)
        self.theta %= 2 * np.pi

        dist = distance + np.random.normal(0.0, self.sd_distance)
        self.x = clamp(self.x + dist * np.cos(self.theta), self.map.bottom_left[0], self.map.top_right[0])
        self.y = clamp(self.y - dist * np.sin(self.theta), self.map.bottom_left[1], self.map.top_right[1])

    def sense(self, sensor_reading):
        # calculate posterior probability for this particle
        p_robot_at_location = self.map.closest_distance((self.x, self.y), self.theta)
        if p_robot_at_location is None or p_robot_at_location == 0:
            return self.prev_log_prob

        p_z_given_robot_at_location = norm.pdf(sensor_reading, loc=p_robot_at_location, scale=self.sd_sensor)
        return math.log(p_z_given_robot_at_location) + self.prev_log_prob

