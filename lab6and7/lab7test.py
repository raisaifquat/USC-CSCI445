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
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            t = self.time.time()

            if action is not None:
                if action_args is None:
                    action()
                else:
                    action(action_args)

            if start + time_in_sec <= t:
                break

    def sweep_sonar(self, degree, curr_angle: float = None, sleep_time: float = 0.01) -> float:
        if curr_angle is None:
            curr_angle = math.degrees(self.odometry.theta)

        min_dist_to_wall = np.array([self.sonar.get_distance()])

        turn_angle = curr_angle - degree
        print("go to %f" % turn_angle)
        self.servo.go_to(turn_angle)
        self.sleep(sleep_time)
        # self.time.sleep(sleep_time)
        min_dist_to_wall = np.append(min_dist_to_wall, self.sonar.get_distance())

        turn_angle = curr_angle + degree
        print("go to %f" % turn_angle)
        self.servo.go_to(turn_angle)
        self.sleep(sleep_time)
        # self.time.sleep(sleep_time)
        min_dist_to_wall = np.append(min_dist_to_wall, self.sonar.get_distance())

        turn_angle = curr_angle
        print("go to %f" % turn_angle)
        self.servo.go_to(turn_angle)
        self.servo.go_to(turn_angle)
        self.sleep(sleep_time)
        # self.time.sleep(sleep_time)

        return min_dist_to_wall.min()

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

        wait_time = 1.0

        self.servo.go_to(30)
        # self.time.sleep(wait_time)
        self.sleep(wait_time, self.servo.go_to, 30)

        self.servo.go_to(-30)
        # self.time.sleep(wait_time)
        self.sleep(wait_time, self.servo.go_to, -30)

        self.servo.go_to(0)
        # self.time.sleep(wait_time)
        self.sleep(wait_time, self.servo.go_to, 0)

        # print(self.sweep_sonar(30, curr_angle=0, sleep_time=0.5))
