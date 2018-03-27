"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""
import math


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        self.arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()

        self.L1 = 0.4  # dist(joint2, joint4), the lower arm length
        self.L2 = 0.39  # dist(joint2, joint4), the upper arm length
        self.lower_arm_joint_number = 2 - 1
        self.upper_arm_joint_number = 4 - 1
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def go_to_angle(self, angle_lower_arm, angle_upper_arm):
        theta1 = math.radians(angle_lower_arm)
        theta2 = math.radians(angle_upper_arm)

        self.arm.go_to(self.lower_arm_joint_number, theta1)
        self.arm.go_to(self.upper_arm_joint_number, theta2)
        self.x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        self.y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)

        print("Go to {:.2f}, {:.2f} deg, FK: [{:.2f}, {:.2f}, {:.2f}]".format(angle_lower_arm, angle_upper_arm, self.x, self.y, self.z))

    def run(self):
        # self.arm.go_to(3, 45)
        self.go_to_angle(45, -90)
        self.time.sleep(10)
        self.go_to_angle(45, -90)
        self.time.sleep(10)
