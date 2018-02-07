from pyCreate2 import create2
from odometry import Odometry
from p_controller import PController
from pd_controller import PDController

import numpy as np
import matplotlib.pyplot as plt

robotProperties = {
    "diameter_left": 72,
    "diameter_right": 72,
    "wheel_base": 235,
    "encoder_count": 508.8
}


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.odometry = Odometry(
            robotProperties["diameter_left"],
            robotProperties["diameter_right"],
            robotProperties["wheel_base"],
            robotProperties["encoder_count"]
        )
        self.p_controller = PController(k_p=1.0)
        self.pd_controller = PDController(k_p=1.5, k_d=-0.005)

    def run(self):

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        r_speed_arr = np.array([])
        l_speed_arr = np.array([])
        time_arr = np.array([])

        v_left_goal = 100
        v_right_goal = 100
        v_left_update = 0
        v_right_update = 0

        start_time = self.time.time()
        curr_time = start_time
        prev_time = curr_time
        while curr_time - start_time < 2:
            self.create.drive_direct(v_left_update, v_right_update)
            state = self.create.update()
            if state is not None:
                # print(state.__dict__)
                time_elapsed = curr_time - prev_time
                self.odometry.update(state.rightEncoderCounts, state.leftEncoderCounts, time_elapsed)
                # print("[x = %f, y = %f, angle = %f]" % (self.odometry.x,
                #                                         self.odometry.y,
                #                                         np.rad2deg(self.odometry.angle)))
                # print("time_elapsed: %f\nr_speed: %f, l_speed: %f" % (
                #     time_elapsed, self.odometry.r_speed, self.odometry.l_speed
                # ))

                # v_right_update = self.p_controller.update_right(v_right_goal, self.odometry.r_speed)
                # v_left_update = self.p_controller.update_left(v_left_goal, self.odometry.l_speed)
                v_right_update = self.pd_controller.update_right(v_right_goal, self.odometry.r_speed, curr_time)
                v_left_update = self.pd_controller.update_left(v_left_goal, self.odometry.l_speed, curr_time)
                # print("v_right_update: %f, v_left_update: %f\n" % (v_right_update, v_left_update))

                time_arr = np.append(time_arr, curr_time)
                r_speed_arr = np.append(r_speed_arr, self.odometry.r_speed)
                l_speed_arr = np.append(l_speed_arr, self.odometry.l_speed)

            prev_time = curr_time
            curr_time = self.time.time()

        plt.plot(time_arr, r_speed_arr)
        # plt.plot(time_arr, l_speed_arr)
        plt.xlabel('time')
        plt.ylabel('right wheel speed')
        plt.grid()
        plt.show()
        # file = open('output.txt', 'w')
        # file.write("time_arr = np.array([")
        # for i in range(0, len(time_arr)):
        #     file.write("%f, " % time_arr[i])
        # file.write("])\n")
        #
        # file.write("r_speed_arr = np.array([")
        # for i in range(0, len(r_speed_arr)):
        #     file.write("%f, " % time_arr[i])
        # file.write("])")
        # file.close()
