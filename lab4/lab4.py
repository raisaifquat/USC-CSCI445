from pyCreate2 import create2
from p_controller import PController


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.p_controller = PController()

    def run(self):

        self.create.start()
        self.create.safe()

        start_time = self.time.time()
        while self.time.time() - start_time < 10:
            self.create.drive_direct(50, -50)
            state = self.create.update()
            if state is not None:
                print(state.__dict__)
                self.print_odometry(state)
