from mecanum_driver import MecanumDriver

import time
import numpy as np


def main():
    print('initializing motor driver')
    mecanum_driver = MecanumDriver()
    
    def send_action(action):
        print(f"Sending action {action}")
        mecanum_driver.send_action(action)
    	
    def straight(speed):
        send_action(np.array([speed, 0., 0.]))
        
    def strafe(speed):
        send_action(np.array([0., speed, 0.]))
        
    def rotate(ang_vel):
        send_action(np.array([0., 0., ang_vel]))
    
    def custom(x,y,ang_vel):
        send_action(np.array([x,y,ang_vel]))
        
    def zero():
        mecanum_driver.zero_all()
        
    def direct():
        mecanum_driver.test_spin()
        
    try:
        while True:
            user_in = input("Enter a test name and argument: ")
            test_name = user_in.strip().split(" ", 1)
            
            if len(test_name) == 1:
                args = []
            else:
                args = list(map(float, test_name[1].split(" ")))
                
            test_name = test_name[0]
            
            locals()[test_name](*args)
            
    except BaseException as e:
        print(e)
        mecanum_driver.zero_all()
       

    
if __name__ == '__main__':
    main()
        
