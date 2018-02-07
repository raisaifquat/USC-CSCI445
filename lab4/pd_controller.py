class PDController:
    def __init__(self, k_p: float = 0.0, k_d: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max
        self.error = 0.0

        self.k_p = k_p
        pass

    def update(self, goal: float, curr_state: float) -> float:
        self.error = goal - curr_state

        return self.error * self.k_p
