import logging
import os

import cv2
import numpy as np
import pickle
from skimage.feature import hog
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import LinearSVC

logging.basicConfig(format='%(asctime)s : %(message)s', level=logging.DEBUG)
logger = logging.getLogger(__name__)

MODEL_CHECKPOINT = 'data/model.p'

class Params(object):

    def __init__(self, color_space, hog_params, size, nbins, heatmap_threshold):
        self.color_space = color_space
        self.hog_params = hog_params
        self.nbins = nbins
        self.size = size
        self.heatmap_threshold = heatmap_threshold


def spatial(img, size):
    return cv2.resize(img, size).ravel()


def color_histogram(img, bins):
    ch = []
    for i in range(img.shape[2]):
        ch.append(np.histogram(img[:, :, i], bins=bins))
    return np.concatenate((ch[0][0], ch[1][0], ch[2][0]))


def hog_features(img, params):
    output = []
    for ch in range(img.shape[2]):
        feat = hog(img[:,:,ch], **params)
        output.append(feat)
    return output


def extract_features(path, params):
    logger.debug('[extract_features] start...')
    features = []
    for filename in path:
        img = cv2.imread(filename, cv2.IMREAD_COLOR)

        if params.color_space:
            feature_image = cv2.cvtColor(img, params.color_space)
        
        spatial_feat = spatial(feature_image, params.size)
        hist_feat = color_histogram(feature_image, params.nbins)
        hog_feat = np.ravel(hog_features(feature_image, params.hog_params))

        features.append(np.concatenate((spatial_feat, hist_feat, hog_feat)))
    return features

def train(car_features, non_car_features):
    logger.debug('[train] start')

    x = np.vstack((car_features, non_car_features)).astype(np.float64)
    scaler = StandardScaler().fit(x)
    scaled_x = scaler.transform(x)

    y = np.hstack((np.ones(len(car_features)),
                   np.zeros(len(non_car_features))))

    split_params = {
        'test_size': 0.2,
        'random_state': np.random.randint(0, 100)
    }
    x_train, x_test, y_train, y_test = train_test_split(scaled_x, y, **split_params)
    clf = LinearSVC()
    clf.fit(x_train, y_train)
    accuracy = clf.score(x_test, y_test)
    logger.debug('[train] accuracy = %s' % accuracy)

    return clf, scaler
