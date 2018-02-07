class PController:
    def __init__(self, k_p: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max
        self.error = 0.0

        self.k_p = k_p
        pass

    def update(self, goal: float, current_val: float) -> float:
        error = goal - current_val

        self.error = error

        return clamp(current_val + error * self.k_p, self.range_min, self.range_max)


def clamp(value, range_min, range_max):
    return max(min(value, range_max), range_min)
