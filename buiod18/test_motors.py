from mecanum_drive import MecanumDriver
import time

def main():
    mecanum_driver = MecanumDriver()
    mecanum_driver.test_spin(0.5)
    time.sleep(1.0)
    mecanum_driver.zero_all()

if __name__ == '__main__':
    main()