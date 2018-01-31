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

    def get_delta_theta(self, r_encoder_count: int, l_encoder_count: int) -> float:
        return (self.get_delta_r(r_encoder_count) - self.get_delta_l(l_encoder_count)) / self.wheel_base
