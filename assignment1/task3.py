import numpy as np
import matplotlib.pyplot as plt
from typing import List


def mysum2(nums: List[int]) -> int:
    return np.sum(nums).item()


def plotcircle1(r: float = 1.0) -> None:
    x = np.arange(-180.0 * np.pi / 180.0, 180.0 * np.pi / 180.0, 0.025, dtype=np.float)
    # x = np.array(
    #     [0, np.pi / 4.0, np.pi / 2.0, 3.0 * np.pi / 4.0, np.pi, 5.0 * np.pi / 4.0, 3.0 * np.pi / 2.0, 7.0 * np.pi / 4.0,
    #      2.0 * np.pi])

    y = np.vectorize(lambda result: np.sin(result), otypes=[np.float])(x)
    y = np.multiply(y, r)

    x = np.vectorize(lambda result: np.cos(result), otypes=[np.float])(x)
    x = np.multiply(x, r)

    plt.plot(x, y)
    plt.show()


def plotnorm1() -> None:
    pass
