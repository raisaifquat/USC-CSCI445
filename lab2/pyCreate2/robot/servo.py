"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""
from .pwm import Pwm


class Servo:
    def __init__(self, number: int) -> None:
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self.pwm = Pwm(number)
        self.pwm.enable()
        self.full_pulse = 20.0e-3
        self.period = self.full_pulse
        self.frequency = int(1 / self.period)
        # 0.75–2.25 ms high pulse, 20 ms intervals
        # self.pwm.set_frequency(self.frequency)
        self.pwm.set_frequency(self.frequency)

    def __del__(self) -> None:
        # self.pwm.disable()
        pass

    def go_to(self, angle: float) -> None:
        # clamp angle to [-90.0, 90.0]
        angle = -90.0 if angle < -90.0 else angle
        angle = 90.0 if angle > 90.0 else angle

        # percent = (angle + 90) / 180 * self.full_pulse / self.period
        num = 1125 - 375
        percent = (angle + 90) / 180 * num + 375
        percent /= 100.0
        print("angle: " + str(angle) + "\npercent: " + str(percent))
        self.pwm.set_duty_cycle(percent)
