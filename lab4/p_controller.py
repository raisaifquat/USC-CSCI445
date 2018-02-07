from utils import clamp


class PController:
    def __init__(self, k_p: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max
        self.l_error = 0.0
        self.r_error = 0.0

        self.k_p = k_p
        pass

    def update_left(self, goal: float, current_val: float) -> float:
        error = goal - current_val
        self.l_error = error

        return clamp(current_val + error * self.k_p, self.range_min, self.range_max)

    def update_right(self, goal: float, current_val: float) -> float:
        error = goal - current_val
        self.r_error = error

        return clamp(current_val + error * self.k_p, self.range_min, self.range_max)

