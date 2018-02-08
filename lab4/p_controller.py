from utils import clamp


class PController:
    def __init__(self, k_p: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max
        self.l_error = 0.0
        self.r_error = 0.0

        self.k_p = k_p
        pass

    def update(self, error: float) -> float:
        return error * self.k_p

