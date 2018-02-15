import numpy as np
import matplotlib.pyplot as plt

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
