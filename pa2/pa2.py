"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""
import math
import numpy as np


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        self.arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()

        self.L1 = 0.4  # dist(joint2, joint4), the lower arm length
        self.L2 = 0.39  # dist(joint4, joint6), the upper arm length
        self.lower_arm_joint_number = 2 - 1
        self.upper_arm_joint_number = 4 - 1
        self.arm_height_offset = 0.3105
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def go_to_angle(self, angle_lower_arm, angle_upper_arm):
        # angle: < 0 to the right, > 0 to the left
        theta1 = math.radians(angle_lower_arm)
        theta2 = math.radians(angle_upper_arm)

        self.arm.go_to(self.lower_arm_joint_number, math.radians(angle_lower_arm))
        self.arm.go_to(self.upper_arm_joint_number, math.radians(angle_upper_arm))

        # self.x = self.L1 * math.cos(theta1 + math.pi / 2) + self.L2 * math.cos(theta1 + theta2 + math.pi / 2)
        # self.z = self.L1 * math.sin(theta1 + math.pi / 2) + self.L2 * math.sin(theta1 + theta2 + math.pi / 2)
        self.z = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        self.x = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        self.x = -self.x

        print("Go to {:.2f}, {:.2f} deg, FK: [{:.2f}, {:.2f}, {:.2f}]"
              .format(angle_lower_arm, angle_upper_arm, self.x, self.y, self.z + self.arm_height_offset))

    def go_to_position(self, x, y):
        r = get_dist(0, self.arm_height_offset, x, y)
        alpha = math.acos((self.L1 ** 2 + self.L2 ** 2 - r ** 2) / (2 * self.L1 * self.L2))
        beta = math.acos((r ** 2 + self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * r))

        theta2 = math.pi - abs(alpha)
        theta1 = math.atan2(y, x) - abs(beta) - math.pi / 2

        # angle: < 0 to the right, > 0 to the left
        angle_lower_arm = math.degrees(theta1)
        angle_upper_arm = math.degrees(theta2)

        print("Go to [{:.2f}, {:.2f}], IK: [{:.2f}, {:.2f}]"
              .format(x, y, angle_lower_arm, angle_upper_arm))

        self.go_to_angle(angle_lower_arm, angle_upper_arm)

    def draw(self, x, y, color):
        pass

    def run(self):
        self.arm.enable_painting()

        # angle = -90
        self.arm.go_to(5, math.radians(90))
        self.arm.go_to(4, math.radians(-90))
        self.time.sleep(1)

        print("24: -108 to 108")
        print("46: -121 to 121")

        # self.go_to_angle(45, -90)
        # self.time.sleep(10)
        # self.go_to_angle(90, -70)
        # self.time.sleep(10)
        # self.go_to_angle(20, -180)
        # self.time.sleep(10)
        # self.go_to_angle(45, -10)
        self.go_to_position(0, 0)
        self.time.sleep(10)


def get_dist(start_x, start_y, dest_x, dest_y):
    return math.sqrt((dest_x - start_x) ** 2 + (dest_y - start_y) ** 2)
