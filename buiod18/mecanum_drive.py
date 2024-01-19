import numpy as np
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

class MecanumDriver:
    def __init__(self,
                 pca_frequency=1600,
                 whl_radius=0.02,
                 max_whl_speed=20,
                 body_length=0.3,
                 wheel_base=0.2):
        i2c_bus = busio.I2C(board.SCL,board.SDA)
        pca = PCA9685(i2c_bus, address=0x60)
        pca.frequency = pca_frequency

        self.motor1 = motor.DCMotor(pca.channels[9],pca.channels[10])
        self.motor2 = motor.DCMotor(pca.channels[11],pca.channels[12])
        self.motor3 = motor.DCMotor(pca.channels[3],pca.channels[4])
        self.motor4 = motor.DCMotor(pca.channels[5],pca.channels[6])

        self.whl_radius = whl_radius
        self.max_whl_speed = max_whl_speed
        self.body_length = body_length
        self.wheel_base = wheel_base
    
    def test_spin(self,throttle_val):
        self.motor1.throttle = throttle_val
        self.motor2.throttle = throttle_val
        self.motor3.throttle = throttle_val
        self.motor4.throttle = throttle_val


    def send_action(self, action):
        throttles = self._whl_speeds_to_throttle(self._action_to_whl_speeds(action))

        # remember to flip 2 and 4
        self.motor1.throttle = throttles[0]
        self.motor2.throttle = throttles[1]
        self.motor3.throttle = throttles[2]
        self.motor4.throttle = throttles[3]
    
    def zero_all(self):
        self.motor1.throttle = 0
        self.motor2.throttle = 0
        self.motor3.throttle = 0
        self.motor4.throttle = 0

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







