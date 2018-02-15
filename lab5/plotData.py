import numpy as np
import matplotlib.pyplot as plt


def plot_time_vs_angle():
    goal_angle = np.pi / 2

    plt_time_arr = np.genfromtxt('timeOutput.csv', delimiter=',')
    plt_angle_arr = np.genfromtxt('angleOutput.csv', delimiter=',')

    plt.title("Time vs Angle")
    plt.xlabel("Time (in second)")
    plt.ylabel("Angle (in radian)")

    plt.axhline(y=goal_angle, color='r', linestyle='--', label='Target angle')
    plt.plot(plt_time_arr, plt_angle_arr, label='Actual angle')
    plt.legend()
    plt.grid()
    plt.show()


def plot_angle_vs_goal_angle():
    plt_angle_arr = np.genfromtxt('angleOutput.csv', delimiter=',')
    plt_goal_angle_arr = np.genfromtxt('goalAngleOutput.csv', delimiter=',')

    plt.title("Angle vs Goal Angle")
    plt.xlabel("Goal Angle (in radian)")
    plt.ylabel("Angle (in radian)")

    plt.plot(plt_angle_arr, plt_goal_angle_arr, label='Actual angle')
    plt.legend()
    plt.grid()
    plt.show()


plot_angle_vs_goal_angle()
