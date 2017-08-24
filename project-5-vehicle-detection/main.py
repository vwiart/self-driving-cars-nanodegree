import os
from glob import glob

import cv2
from moviepy.editor import VideoFileClip
import pickle

from classifier import extract_features, Params, train
from process import Process


def get_features(dataset, params, filename):
    if os.path.exists(filename):
        with open(filename, mode='rb') as f:
            return pickle.load(f)
    features = extract_features(dataset, params)
    with open(filename, 'wb') as f:
        pickle.dump(features, f)
    return features


def get_model(car_features, non_car_features, filename):
    if os.path.exists(filename):
        with open(filename, mode='rb') as f:
            data = pickle.load(f)
            clf = data['clf']
            scaler = data['scaler']
            return clf, scaler
    clf, scaler = train(car_features, non_car_features)
    with open(filename, mode='wb') as f:
        pickle.dump({
            'clf': clf,
            'scaler': scaler,
        }, f)
    return clf, scaler


def test_process(clf, scaler, params):
    images = glob('test_images/test*.jpg')
    for filename in images:
        img = cv2.imread(filename, cv2.IMREAD_COLOR)
        process = Process(clf=clf, scaler=scaler, params=params)
        img = process.process(img)
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    cars = glob('data/vehicles/**/*.png')
    non_cars = glob('data/non-vehicles/**/*.png')
    
    hog_params = {
        'orientations': 9,
        'pixels_per_cell': (8, 8),
        'cells_per_block': (2, 2),
        'transform_sqrt': False,
        'visualise': False,
        'feature_vector': True,
    }

    params = Params(color_space=cv2.COLOR_RGB2YCrCb,
                    hog_params=hog_params,
                    nbins=32,
                    size=(32, 32),
                    heatmap_threshold=4)

    car_features = get_features(cars, params, 'data/car_features.p')
    non_car_features = get_features(non_cars, params, 'data/non_car_features.p')
    clf, scaler = get_model(car_features, non_car_features, 'data/model.p')

    process = Process(clf=clf, scaler=scaler, params=params)
    filename = 'project_video.mp4'
    clip = VideoFileClip(filename)
    res = clip.fl_image(process.process)
    res.write_videofile('output_images/' + filename, audio=False)


if __name__ == '__main__':
    main()
