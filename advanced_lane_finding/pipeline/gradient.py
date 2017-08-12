import cv2
import numpy as np

class Sobel(object):
    
    def __init__(self, threshold_min, threshold_max, kernel=None):
        self.kernel = kernel
        self.threshold_min = threshold_min
        self.threshold_max = threshold_max

    def activate(self, val):
        binary = np.zeros_like(val)
        binary[(val >= self.threshold_min) & (val <= self.threshold_max)] = 1
        return binary

class Gradient(object):

    def __init__(self, abs_sobel, l_sobel, b_sobel):
        self.abs_sobel = abs_sobel
        self.l_sobel = l_sobel
        self.b_sobel = b_sobel


    def process(self, img):
        luv = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        lab = cv2.cvtColor(img, cv2.COLOR_RGB2Lab)
        l_channel = luv[:, :, 0]
        b_channel = lab[:,:, 2]

        # Threshold lightness
        l_binary = self.l_sobel.activate(l_channel)
        # Threshold yellow
        b_binary = self.b_sobel.activate(b_channel)


        combined_binary = np.zeros_like(l_binary)
        combined_binary[(l_binary == 1) | (b_binary == 1)] = 1
        return combined_binary
