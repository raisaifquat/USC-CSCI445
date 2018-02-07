import numpy as np


class Odometry:
    def __init__(
            self,
            diameter_left: float,
            diameter_right: float,
            wheel_base: float,
            encoder_count: float
    ):
        self.diameter_left = diameter_left
        self.diameter_right = diameter_right
        self.wheel_base = wheel_base
        self.encoder_count = encoder_count

        self.r_distance = 0.0
        self.l_distance = 0.0
        self.angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.prev_r_count = 0
        self.prev_l_count = 0

    def get_delta_r(self, r_encoder_count: int) -> float:
        return self.diameter_right * r_encoder_count / self.wheel_base * np.pi

    def get_delta_l(self, l_encoder_count: int) -> float:
        return self.diameter_left * l_encoder_count / self.wheel_base * np.pi

    def get_delta_d(self, r_encoder_count: int = 0, l_encoder_count: int = 0,
                    delta_r: float = None, delta_l: float = None) -> float:
        if delta_r is None:
            delta_r = self.get_delta_r(r_encoder_count)
        if delta_l is None:
            delta_l = self.get_delta_l(l_encoder_count)

        return (delta_r + delta_l) / 2.0

    def get_delta_theta(self, r_encoder_count: int = 0, l_encoder_count: int = 0,
                        delta_r: float = None, delta_l: float = None) -> float:
        if delta_r is None:
            delta_r = self.get_delta_r(r_encoder_count)
        if delta_l is None:
            delta_l = self.get_delta_l(l_encoder_count)

        return (delta_r - delta_l) / self.wheel_base

    def update(self, r_encoder_count: int = 0, l_encoder_count: int = 0):

        delta_r = self.get_delta_r(r_encoder_count - self.prev_r_count)
        delta_l = self.get_delta_l(l_encoder_count - self.prev_l_count)
        self.prev_r_count = r_encoder_count
        self.prev_l_count = l_encoder_count

        delta_theta = self.get_delta_theta(delta_r=delta_r, delta_l=delta_l)
        delta_d = self.get_delta_d(delta_r=delta_r, delta_l=delta_l)
        
        self.x += delta_d * np.cos(self.angle)
        self.y += delta_d * np.sin(self.angle)
        self.angle += delta_theta
