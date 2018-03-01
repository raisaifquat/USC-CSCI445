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

    def go_to_goal(self, goal_x, goal_y):
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
            # print("[{%.4f},{%.4f},{%.4f}]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

            # improved version 2: fuse with velocity controller
            distance = self.dist_to_goal(goal_x, goal_y)
            output_distance = self.pidDistance.update(0, distance, self.time.time())
            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

    def dist_to_goal(self, goal_x, goal_y):
        return math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

    def dist_to_wall(self):
        return self.sonar.get_distance()

    def follow_wall(self, goal_dist, base_speed: float = 100.0):
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        distance = self.sonar.get_distance()
        if distance is not None:
            # print(distance)
            output = self.pd_controller.update(distance, goal_dist, self.time.time())
            self.create.drive_direct(int(base_speed - output), int(base_speed + output))
            self.time.sleep(0.01)

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
        dist_threshold = 0.1
        wall_threshold = 0.4
        distance_offset = 0.1

        for point in waypoints:
            goal_x = point[0]
            goal_y = point[1]
            base_speed = 100

            print("Going to @{%.4f, %.4f}" % (goal_x, goal_y))

            while self.dist_to_goal(goal_x, goal_y) > dist_threshold:

                dist_to_wall = self.dist_to_wall()
                while dist_to_wall is not None and dist_to_wall > wall_threshold:
                    # print(dist_to_wall)
                    self.go_to_goal(goal_x, goal_y)
                    dist_to_wall = self.dist_to_wall()

                original_dist_to_goal = self.dist_to_goal(goal_x, goal_y)
                sonar_angle = math.degrees(self.odometry.theta)
                print("dist to goal = %f\n" % original_dist_to_goal)
                while self.dist_to_goal(goal_x, goal_y) - original_dist_to_goal < distance_offset:
                    self.follow_wall(wall_threshold + 0.1, base_speed=base_speed)
                    # self.create.drive_direct(0, 0)
                    self.servo.go_to(-(math.degrees(self.odometry.theta) - sonar_angle))
                    self.time.sleep(0.5)
                    print("prev angle %f" % sonar_angle)
                    print("current angle %f" % math.degrees(self.odometry.theta))
                    sonar_angle = math.degrees(self.odometry.theta)

            print("Arrived @[{%.4f},{%.4f},{%.4f}]\n" % (
                self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
