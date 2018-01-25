import numpy as np
import matplotlib.pyplot as plt
from typing import List


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()

    def run(self):
        data = np.zeros(100)
        for i in range(0, 100):
            data[i] = self.sonar.get_distance()
            print(data[i])
            # self.time.sleep(0.1)
            self.time.sleep(1 / 20e-3)


def plotHist(data: List[float] = None, num_bins: int = 20) -> None:
    if data is None:
        data = np.random.normal(0, 0.5, 10000)

    plt.hist(data, bins=num_bins, edgecolor='black', linewidth=1.0)
    plt.show()


def showStat(data: List[float]) -> None:
    print("Mean: " + np.mean(data))
    print("SD: " + np.std(data))
