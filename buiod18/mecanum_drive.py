import numpy as np
import busio
import board
from adafruit_motorkit import MotorKit

class MecanumDriver:
    def __init__(self,
                 pwm_frequency=1600,
                 whl_radius=0.02,
                 max_whl_speed=20,
                 body_length=0.3,
                 wheel_base=0.2):
        i2c_bus = busio.I2C(board.SCL,board.SDA)
        self.kit = MotorKit(address=0x60, i2c=i2c_bus, pwm_frequency=pwm_frequency)
        self.whl_radius = whl_radius
        self.max_whl_speed = max_whl_speed
        self.body_length = body_length
        self.wheel_base = wheel_base

    def send_action(self, action):
        throttles = self._whl_speeds_to_throttle(self._action_to_whl_speeds(action))

        # remember to flip 2 and 4
        self.kit.motor1.throttle = throttles[0]
        self.kit.motor2.throttle = throttles[1]
        self.kit.motor3.throttle = throttles[2]
        self.kit.motor4.throttle = throttles[3]
    
    def zero_all(self):
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0
        self.kit.motor3.throttle = 0
        self.kit.motor4.throttle = 0

    def _action_to_whl_speeds(self, action):
        mag = np.linalg.norm(action[:2]) / self.whl_radius
        heading = np.arctan2(action[1], action[0])

        no_turn_speeds = np.array([
            np.sin(heading + np.pi / 4),
            np.sin(heading + 3 * np.pi / 4),
            np.sin(heading + np.pi / 4),
            np.sin(heading + 3 * np.pi / 4)
        ]) * mag

        turn_speeds = np.sqrt(2) / 2 * action[2] * np.sqrt((self.body_length / 2) ** 2 + (self.wheel_base / 2) ** 2) \
                    * self.wheel_base / self.body_length * np.array([1, -1, 1, -1])

        return no_turn_speeds + turn_speeds

    def _whl_speeds_to_throttle(self, whl_speeds):
        return np.clip(whl_speeds / self.max_whl_speed,-1,1)* np.array([1, -1, 1, -1])







