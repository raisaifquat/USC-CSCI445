"""
Example to move robot forward for 10 seconds
Use "python3 run.py [--sim] example1" to execute
"""
from my_robot import MyRobot


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.my_robot = MyRobot(None, self.create, self.time)

    def run(self):
        self.create.start()
        self.create.safe()

        # move forward
        self.my_robot.forward(5 * 0.1)

        # left turn (in place)
        self.my_robot.turn_left(2)

        # wait
        self.my_robot.wait(2)

        # right turn (in place)
        self.my_robot.turn_right(2)

        # wait
        self.my_robot.wait(2)

        # move forward
        self.my_robot.forward(2 * 0.1)

        # move forward while turning
        self.my_robot.move(0.2, 0.1, 7.5)

        # move forward
        self.my_robot.forward(5 * 0.1)

        # move backwards
        self.my_robot.backward(5 * 0.1)

        # stop
        self.my_robot.wait(3)

        self.create.stop()
