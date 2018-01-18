import numpy as np
import matplotlib.pyplot as plt
from typing import List


def mysum2(nums: List[int]) -> int:
    result = np.sum(nums).item()

    print(result)
    return result


def plotcircle1(r: float = 1.0) -> None:
    x = np.arange(0.0, 2.0 * np.pi, 0.025, dtype=np.double)

    y = np.sin(x)
    y = np.multiply(y, r)

    x = np.cos(x)
    x = np.multiply(x, r)

    plt.plot(x, y)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.show()


def plotnorm1(num_bins: int = 20) -> None:
    s = np.random.normal(0, 0.5, 10000)

    plt.hist(s, bins=num_bins, edgecolor='black', linewidth=1.0)
    plt.show()
