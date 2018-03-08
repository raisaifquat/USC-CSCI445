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

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        waypoints = np.array([
            [2.0, 0.0],
            [2.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.0]
        ])
        # waypoints = np.array([[2, 0]])
        for point in waypoints:
            goal_x = point[0]
            goal_y = point[1]
            # base_speed = 100

            print("Going to @{%.4f, %.4f}" % (goal_x, goal_y))
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            while distance > 0.05:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    # print("[{%.4f},{%.4f},{%.4f}]" % (self.odometry.x, self.odometry.y,
                    #                                   np.rad2deg(self.odometry.theta))
                    #       )

                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # improved version 2: fuse with velocity controller
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

            print("Arrived @[{%.4f},{%.4f},{%.4f}]\n" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
