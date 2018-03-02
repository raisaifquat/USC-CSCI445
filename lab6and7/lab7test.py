from pyCreate2 import create2
import math
import numpy as np

import odometry
import pd_controller2
import pid_controller

import matplotlib

# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pd_controller = pid_controller.PIDController(1000, 0, 100, [-75, 75], [-200, 200], is_angle=False)

    def sleep(self, time_in_sec, action=None, action_args=None):
        result = np.array([])
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            t = self.time.time()

            if action is not None:
                if action_args is None:
                    result = np.append(result, action())
                else:
                    action(action_args)

            if start + time_in_sec <= t:
                break

            return None if result.size == 0 else result.min()

    def sweep_sonar(self, degree, sleep_time: float = 1.0) -> float:
        curr_angle = math.degrees(self.odometry.theta)

        # min_dist_to_wall = np.array([self.sonar.get_distance()])
        print(self.sonar.get_distance())
        self.sleep(0.1)

        self.servo.go_to(curr_angle - degree)
        # self.sleep(sleep_time)
        self.sleep(sleep_time, self.sonar.get_distance)
        # min_dist_to_wall = np.append(min_dist_to_wall, self.sonar.get_distance())
        self.sleep(0.1)
        print(self.sonar.get_distance())
        self.sleep(0.1)

        self.servo.go_to(curr_angle + degree)
        # self.sleep(sleep_time)
        self.sleep(sleep_time, self.sonar.get_distance)
        # min_dist_to_wall = np.append(min_dist_to_wall, self.sonar.get_distance())
        self.sleep(0.1)
        print(self.sonar.get_distance())
        self.sleep(0.1)

        self.servo.go_to(curr_angle)
        # self.sleep(sleep_time)
        self.sleep(sleep_time, self.sonar.get_distance)

        # return min_dist_to_wall.min()
        return 0

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.servo.go_to(0)
        self.time.sleep(2)

        # print(self.sonar.get_distance())

        curr_angle = 0.0
        degree = 15
        wait_time = 0.5

        # turn_angle = curr_angle + degree
        # self.servo.go_to(turn_angle)
        # self.time.sleep(wait_time)
        # # self.sleep(wait_time, self.servo.go_to, turn_angle)
        #
        # turn_angle = curr_angle - degree
        # self.servo.go_to(turn_angle)
        # self.time.sleep(wait_time)
        # # self.sleep(wait_time, self.servo.go_to, turn_angle)
        #
        # turn_angle = curr_angle
        # self.servo.go_to(turn_angle)
        # self.time.sleep(wait_time)
        # # self.sleep(wait_time, self.servo.go_to, turn_angle)

        self.time.sleep(2)

        print("----------------")
        print("%f" % self.sweep_sonar(degree, sleep_time=wait_time))
