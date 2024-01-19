from mecanum_drive import MecanumDriver
from mecanum_motorkit import MecanumDriver_MotorKit
import time

def main():
    print('initializing motor driver')
    mecanum_driver = MecanumDriver_MotorKit()
    print('spinning motors')
    mecanum_driver.test_spin(0.5)
    time.sleep(1.0)
    print('stopping')
    mecanum_driver.zero_all()

if __name__ == '__main__':
    main()
