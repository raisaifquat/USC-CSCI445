import numpy as np


class Odometry:
    def __init__(self, diameter_left, diameter_right, wheel_base, encoder_count):
        self.diameter_left = diameter_left
        self.diameter_right = diameter_right
        self.wheel_base = wheel_base
        self.encoder_count = encoder_count

    def get_delta_r(self):
        pass

    def get_delta_l(self):
        pass

    def get_delta_d(self):
        pass

    def get_delta_theta(self):
        pass
