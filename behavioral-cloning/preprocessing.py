import csv
import os

import cv2
import numpy as np
from numpy.random import shuffle
import pickle

CSV_FILENAME= 'driving_log.csv'


def load_image(filename):
    """loads an image with opencv2."""
    separator = '\\' if os.name in ['nt'] else '/'
    chunk = filename.split(separator)[-3:]
    image = cv2.imread("/".join(chunk))
    return image


def batches(dirname, data, batch_size, steering_correction):
    num_samples = len(data)
    while True:
        shuffle(data)
        for offset in range(0, num_samples, batch_size):
            batch = data[offset: offset + batch_size]

            images = []
            angles = []
            for row in batch:
                flip_steering = lambda s: -1.0 * s
                
                steering = [float(row[3]),
                            float(row[3]) + steering_correction,
                            float(row[3]) - steering_correction]

                for i in range(0, 3):
                    img = load_image(row[i])
                    images.append(img)
                    images.append(cv2.flip(img, 1))

                    angles.append(steering[i])
                    angles.append(flip_steering(steering[i]))                

            yield np.array(images), np.array(angles)


def parse(dirname):
    """parse a csv file."""
    data = []
    with open(dirname + CSV_FILENAME, 'rt') as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(row)
    return data