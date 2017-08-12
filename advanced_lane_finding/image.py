import cv2
import matplotlib.pyplot as plt
import numpy as np


def load(filename):
    return cv2.imread(filename)


def display(img, cmap=None):
    plt.imshow(img, cmap=cmap)
    plt.show()


def save(img, path):
    cv2.imwrite(path, img)
