import glob
import os
import numpy as np
import cv2 as cv
import click
import json
from controller import Driver


if __name__ == '__main__':
    driver = Driver();

    imgL = cv.imread('./hi/left/000000.png');
    imgR = cv.imread('./hi/right/000000.png');

    die = driver.get_depth_map(imgL, imgR);
    print("die", die);











