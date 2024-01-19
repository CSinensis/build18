import numpy as np
from enum import Enum
import cv2 as cv
import matplotlib
import matplotlib.pyplot as plt
import time

def now_ms():
    return time.time() * 1e-3


class Driver:

    class StrafeDir(Enum):
        LEFT = 0
        RIGHT = 1

        def other(self):
            return StrafeDir((self.value + 1) % 2)


    class Vibe(Enum):
        SEEKING = 0
        STRAFING = 1
        SPINNING = 2


    class State:

        def __init__(self):
            self.vibe = Driver.Vibe.SEEKING
            self.strafe_dir = Driver.INIT_STRAFE_DIR.other()
            self.strafe_switch_time = now_ms()

        def set_vibe(self, vibe):
            if vibe == Driver.Vibe.STRAFING and self.vibe != Driver.Vibe.STRAFING:
                self.strafe_dir = self.strafe_dir.other()
                self.strafe_switch_time = now_ms() - Driver.STRAFE_TIMEOUT / 2

            self.vibe = vibe

        def switch_strafe(self):
            # toggle strafe dir
            self.strafe_dir = self.strafe_dir.other()
            self.strafe_switch_time = now_ms()


    BUBBLE_RADIUS = 5
    STRAFE_TIMEOUT = 8000
    INIT_STRAFE_DIR = StrafeDir.LEFT
    FOV = np.radians(90)
    TRACKING_SPEED = 0.25  # m/s
    STRAFE_SPEED = 0.1
    SPIN_ANG_VEL = 1  # rad/s
    ANG_VEL_K_P = 1


    def __init__(self):
        self.state = Driver.State()
        self.stereo = cv.StereoSGBM_create(
            minDisparity = 1,
            numDisparities = 128,
            blockSize = 3,
            P1 = 8*3**2,
            P2 = 32*3**2,
            disp12MaxDiff = 1,
            preFilterCap = 63,
            uniquenessRatio = 10,
            speckleWindowSize = 100,
            speckleRange = 32,
            mode = 0
        )

    def step(self, imgL, imgR):
        vest_heading = self.get_vest_heading(imgL, imgR)

        if vest_heading is None:
            self.state.set_vibe(Driver.Vibe.SPINNING)
            action = self.get_spin_action()
        
        elif self.get_heading_feasibility(vest_heading):
            self.state.set_vibe(Driver.Vibe.SEEKING)
            action = self.get_follow_heading_action(vest_heading)
        
        else:
            self.state.set_vibe(Driver.Vibe.STRAFING)
            if self.state.strafe_switch_time >= Driver.STRAFE_TIMEOUT:
                self.state.switch_strafe()

            action = self.get_strafe_action()

        return action


    def get_vest_heading(self, imgL, imgR):
        imgs = [imgL, imgR]
        centroids = []

        img_xlen = imgL.shape[0]

        for img in imgs:
            hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            
            #TODO: tune these (highlight color ish)
            color_threshold_low = np.array([28, 36, 55])
            color_threshold_high = np.array([50, 255, 243]) 

            #(hMin = 28 , sMin = 36, vMin = 55), 
            #(hMax = 50 , sMax = 255, vMax = 243)

            mask = cv.inRange(hsv_img, color_threshold_low, color_threshold_high)
            masked_img = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
            masked_img = cv.cvtColor(masked_img, cv.COLOR_HSV2RGB)
            gray = cv.cvtColor(gray, cv.COLOR_RGB2GRAY)
            contours,hierarchy = cv.findContours(gray,cv.RETR_EXTERNAL, 
                                                 cv.CHAIN_APPROX_SIMPLE)

            cnt_area = []
            for i in range(len(contours)):
                cnt_area.append(cv.contourArea(contours[i]))
            max_ind = cnt_area.index(max(cnt_area))

            x,y,w,h = cv.boundingRect(contours[max_ind])

            centroids.append((x+w//2, y+h//2))
        
        true_centroid = (centroids[0]+centroids[1])/2
        horz_dist = (imgL.shape[0] - true_centroid[0])

        heading = horz_dist / img_xlen * Driver.FOV

        return heading


    def get_depth_map(self, imgL, imgR):
        #load calibration parameters from file
        calib = np.load("calib.npz")
        
        #obtain undistorted and rectified unpacked left/right images
        leftMapX = calib["leftMapX"]
        leftMapY = calib["leftMapY"]

        rightMapX = calib["rightMapX"]
        rightMapY = calib["rightMapY"]

        print("left map x", leftMapX);
        print("left map y", leftMapY);
        print("right map x", rightMapX);
        print("right map y", rightMapY);        

        #undistort images with calib parameters
        fixed_left = cv.remap(imgL, leftMapX, leftMapY, interpolation=cv.INTER_LINEAR)
        fixed_right = cv.remap(imgR, rightMapX, rightMapY, interpolation=cv.INTER_LINEAR)

        #make s
        gray_left = cv.cvtColor(fixed_left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(fixed_right, cv.COLOR_BGR2GRAY)

        plt.imshow(gray_left, cmap="gray");

        plt.show()
        plt.imshow(gray_right, cmap="gray");

        plt.show()        
        dd_map = self.stereo.compute(gray_left, gray_right)

        plt.imshow(dd_map, cmap="gray");
        plt.show()            

        #for displaying, note to:
        #dd_map = dd_map * 255/(dd_map.max() - dd_map.min())
        #thn display it to see and verify

        #TODO: check the shape of this, I think it's 2d?
        return dd_map


    def get_heading_feasibility(self, heading, imgL, imgR):
        pass
    
    def get_strafe_action(self):
        strafe_funs = {
            Driver.StrafeDir.LEFT: self.get_strafe_left_action,
            Driver.StrafeDir.RIGHT: self.get_strafe_right_action
        }

        return strafe_funs[self.state.strafe_dir]

    def get_strafe_left_action(self):
        return np.array([0., -Driver.STRAFE_SPEED, 0.])

    def get_strafe_right_action(self):
        return np.array([0., Driver.STRAFE_SPEED, 0.])

    def get_follow_heading_action(self, heading):
        return np.array([
            np.cos(heading) * Driver.TRACKING_SPEED, 
            np.sin(heading) * Driver.TRACKING_SPEED, 
            Driver.ANG_VEL_K_P * heading
        ])
    
    def get_spin_action(self):
        return np.array([0., 0., Driver.SPIN_ANG_VEL])


def send_action():
    pass
    

if __name__ == "__main__":

    driver = Driver()

    while True:
        try:
            # TODO: capture images
            imgL, imgR = None, None

            send_action(driver.step())

        except KeyboardInterrupt:
            # TODO: maybe cleanup
            break

    