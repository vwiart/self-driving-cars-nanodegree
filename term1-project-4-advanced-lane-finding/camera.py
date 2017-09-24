import os

import cv2
import glob
from matplotlib import pyplot as plt
import numpy as np
import pickle


class Camera(object):
    
    def calibrate(self, images):
        """Calibrate the camera using a list of chessboard images."""
        checkpoint = 'checkpoint/calibration.p'
        if os.path.exists(checkpoint):
            with open(checkpoint, mode='rb') as f:
                data = pickle.load(f)
                self.matrix = data['camera_matrix']
                self.distorsion_coef = data['distorsion_coef']
                return
        imgpoints = []
        objpoints = []

        objp = np.zeros((9*6, 3), np.float32)
        objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

        for filename in images:
            img = cv2.imread(filename)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
            if not ret:
                continue

            imgpoints.append(corners)
            objpoints.append(objp)

        ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints,
                                                imgpoints,
                                                gray.shape[::-1],
                                                None,
                                                None)
        if not ret:
            raise CameraException("Unable to calibrate camera")

        with open(checkpoint, mode='wb') as f:
            pickle.dump({
                "camera_matrix": mtx,
                "distorsion_coef": dist
            }, f)
        
        self.matrix = mtx
        self.distorsion_coef = dist
    
    def undistort(self, img):
        return cv2.undistort(img,
                         self.matrix,
                         self.distorsion_coef,
                         None,
                         self.matrix)

class CameraException(Exception):
    pass
