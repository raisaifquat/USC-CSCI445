"""
Sample Code for Lab2 for testing the sonar
Use "run.py [--sim] lab2_sonar_test" to execute
"""


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()

    def run(self):
        while True:
            print(self.sonar.get_distance())
            self.time.sleep(0.1)
