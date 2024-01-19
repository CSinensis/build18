import glob
import os
import numpy as np
import cv2
import click
import json
from json import JSONEncoder
#obtained ferom https://github.com/bartoszptak/Depther/blob/master/2_calibrate.pyd




class Calibrator:
    def __init__(self, imageSize, cb_shape, cb_size):
        self.cb_shape = tuple([int(x) for x in cb_shape.split('x')])
        self.pattern_points = np.zeros((np.prod(self.cb_shape), 3), np.float32)
        self.pattern_points[:, :2] = np.indices(self.cb_shape).T.reshape(-1, 2)
        self.pattern_points *= cb_size

        self.imageSize = tuple([int(x) for x in imageSize.split('x')])
        self.alpha = -1

        self.term = (cv2.TERM_CRITERIA_EPS +
                     cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.arrays = None
        self.calibration = None

    def read_images(self, dir):
        assert os.path.isdir(dir+'/left') and os.path.isdir(dir+'/right')

        def find_corners(p):
            img = cv2.imread(p, 0)
            img = cv2.resize(img, self.imageSize)
            ret, corners = cv2.findChessboardCorners(
                img, self.cb_shape, cv2.CALIB_CB_FAST_CHECK)
            if ret and img.shape[::-1] == self.imageSize:
                cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), self.term)
                return [os.path.basename(p), self.pattern_points, corners]

        arr_left = np.array([find_corners(p)
                             for p in sorted(glob.glob(f"{dir}/left/*.png"))])
        arr_left = arr_left[arr_left != None][0]

        arr_right = np.array([find_corners(p)
                              for p in sorted(glob.glob(f"{dir}/right/*.png"))])
        arr_right = arr_right[arr_right != None][0]

        all_names = sorted(list(set(arr_left[:, 0]) & set(arr_right[:, 0])))

        def get_intersection(arr, all_names):
            return arr[np.isin(arr[:, 0], all_names)]

        arr_left = get_intersection(arr_left, all_names)
        arr_right = get_intersection(arr_right, all_names)

        self.arrays = [arr_left, arr_right]
        print(f'Found {len(arr_left)} images with chessboard')

    def calibrate_cameras(self):
        assert self.arrays
        leftCameraMatrix, leftDistortionCoefficients = cv2.calibrateCamera(
            self.arrays[0][:, 1], self.arrays[0][:, 2], self.imageSize, None, None)[1:3]
        rightCameraMatrix, rightDistortionCoefficients = cv2.calibrateCamera(
            self.arrays[0][:, 1], self.arrays[1][:, 2], self.imageSize, None, None)[1:3]

        rot_matrix, trans_vector = cv2.stereoCalibrate(
            self.arrays[0][:, 1], self.arrays[0][:, 2], self.arrays[1][:, 2],
            leftCameraMatrix, leftDistortionCoefficients,
            rightCameraMatrix, rightDistortionCoefficients,
            self.imageSize, flags=cv2.CALIB_FIX_INTRINSIC, criteria=self.term)[5:7]

        (leftRectification, rightRectification, leftProjection, rightProjection,
        dispartityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(
            leftCameraMatrix, leftDistortionCoefficients,
            rightCameraMatrix, rightDistortionCoefficients,
            self.imageSize, rot_matrix, trans_vector,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=self.alpha)
        
        
        leftMapX, leftMapY = cv2.initUndistortRectifyMap(
                leftCameraMatrix, leftDistortionCoefficients, leftRectification,
                leftProjection, self.imageSize, cv2.CV_32FC1)
        rightMapX, rightMapY = cv2.initUndistortRectifyMap(
                rightCameraMatrix, rightDistortionCoefficients, rightRectification,
                rightProjection, self.imageSize, cv2.CV_32FC1);
    
        np.savez_compressed('calib.npz', imageSize=self.imageSize,
            leftMapX=leftMapX, leftMapY=leftMapY, leftROI=leftROI,
            rightMapX=rightMapX, rightMapY=rightMapY, rightROI=rightROI)





def main(dir, size, cb_shape, cb_size):
    #cb size is size of square
    #cb shape
    #
    calibr = Calibrator(size, cb_shape, cb_size)
    calibr.read_images(dir)
    calibr.calibrate_cameras()


if __name__ == "__main__":
    main('./testCalibImgs', '1280x720', '7x6', 0.0417);
