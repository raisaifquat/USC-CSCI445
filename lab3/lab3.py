"""
Sample Code for Lab3
Use "run.py [--sim] lab3" to execute
"""
from pyCreate2 import create2
from my_robot import MyRobot

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
        self.my_robot = MyRobot(None, self.create, self.time)

    def run(self):
        def move_robot(is_print: bool = False):
            # move forward
            self.my_robot.forward(5 * 0.1, is_print=is_print)

            # left turn (in place)
            self.my_robot.turn_left(2, is_print=is_print)

            # wait
            self.my_robot.wait(2, is_print=is_print)

            # right turn (in place)
            self.my_robot.turn_right(2, is_print=is_print)

            # wait
            self.my_robot.wait(2, is_print=is_print)

            # move forward
            self.my_robot.forward(2 * 0.1, is_print=is_print)

            # move forward while turning
            self.my_robot.move(0.2, 0.1, 7.5, is_print=is_print)

            # move forward
            self.my_robot.forward(5 * 0.1, is_print=is_print)

            # move backwards
            self.my_robot.backward(5 * 0.1, is_print=is_print)

            # stop
            self.my_robot.wait(3, is_print=is_print)

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        while True:
            move_robot(True)
            # state = self.create.update()
            # if state is not None:
            #     print(state.__dict__)
