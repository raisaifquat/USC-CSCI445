import numpy as np
import lab8_map
from scipy.stats import norm
import math


class Particle:
    def __init__(self, create, x, y, prev_prob, theta, sd_sensor, sd_distance, sd_direction, go_to_angle, sleep):
        self.create = create

        # turn and move function
        self.go_to_angle = go_to_angle
        self.sleep = sleep

        # coordination
        self.x = x
        self.y = y
        self.prev_prob = prev_prob
        self.theta = theta

        # constant
        self.sd_sensor = sd_sensor
        self.sd_distance = sd_distance
        self.sd_direction = sd_direction

        # map
        self.map = lab8_map.Map("lab8_map.json")

    def move(self, turn, distance):
        # if (distance < 0):
        #     raise ValueError('robot cannot move backwards')

        # self.theta = self.theta + turn + np.random.normal(0.0, self.sd_direction)
        self.theta = turn + np.random.normal(0.0, self.sd_direction)
        self.theta %= 2 * np.pi

        dist = distance + np.random.normal(0.0, self.sd_distance)
        self.x = self.x + dist * np.cos(self.theta)
        self.y = self.y - dist * np.sin(self.theta)

        if turn != 0:
            self.go_to_angle(self.theta)
        self.create.drive_direct(100, 100)
        self.sleep(dist / 100)

    def sense(self, sensor_reading):
        # calculate posterior probability for this particle
        p_robot_at_location = self.map.closest_distance((self.x, self.y), self.theta)

        p_z_given_robot_at_location = norm.pdf(sensor_reading, loc=p_robot_at_location, scale=self.sd_sensor)
        return math.log(p_z_given_robot_at_location) + math.log(self.prev_prob)

