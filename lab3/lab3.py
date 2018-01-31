"""
Sample Code for Lab3
Use "run.py [--sim] lab3" to execute
"""
from pyCreate2 import create2
from my_robot import MyRobot


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.my_robot = MyRobot(None, self.create.drive_direct, self.time.sleep)

    def run(self):
        def move_robot(state = None):
            def print_func():
                if state is not None:
                    print(state)

            # move forward
            self.my_robot.forward(5 * 0.1)
            print_func()

            # left turn (in place)
            self.my_robot.turn_left(2)
            print_func()

            # wait
            self.my_robot.wait(2)
            print_func()

            # right turn (in place)
            self.my_robot.turn_right(2)
            print_func()

            # wait
            self.my_robot.wait(2)
            print_func()

            # move forward
            self.my_robot.forward(2 * 0.1)
            print_func()

            # move forward while turning
            self.my_robot.move(0.2, 0.1, 7.5)
            print_func()

            # move forward
            self.my_robot.forward(5 * 0.1)
            print_func()

            # move backwards
            self.my_robot.backward(5 * 0.1)
            print_func()

            # stop
            self.my_robot.wait(3)
            print_func()

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        while True:
            state = self.create.update()
            if state is not None:
                move_robot(state.__dict__)
                # print(state.__dict__)
