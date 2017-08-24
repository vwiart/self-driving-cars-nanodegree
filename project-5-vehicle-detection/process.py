import cv2
import numpy as np
from scipy.ndimage.measurements import label

from classifier import hog_features, spatial, color_histogram


class Heatmap(object):

    def __init__(self, y_start, y_stop, scale, params, pixel_per_cell=8,
                 cell_per_block=2, cell_per_steps=2, size=(64, 64)):
        self.y_start = y_start
        self.y_stop = y_stop
        self.scale = scale
        self.params = params
        self.pixel_per_cell = pixel_per_cell
        self.cell_per_block = cell_per_block
        self.window = 64
        self.cell_per_steps = cell_per_steps
        self.size = size
        self.nblocks_per_window = self.window // self.pixel_per_cell - 1

    def region_of_interest(self, img, x_start):
        return img[self.y_start:self.y_stop, x_start:, :]

    def steps(self, x):
        blocks = x // self.pixel_per_cell - self.cell_per_block + 1
        return (blocks - self.nblocks_per_window) // self.cell_per_steps

    def process(self, img, svc, X_scaler):
        heatmap = np.zeros_like(img[:, :, 0])

        xstart = img.shape[1] // 2
        roi = img[self.y_start:self.y_stop, xstart:, :]

        scale = lambda x: np.int(x / self.scale)
        height, width, _ = roi.shape
        scaled_height, scaled_width = scale(height), scale(width)

        roi = cv2.cvtColor(roi, cv2.COLOR_RGB2YCrCb)
        roi = cv2.resize(roi, (scaled_width, scaled_height))

        hog_params = self.params.hog_params
        hog_params['feature_vector'] = False
        hog = hog_features(roi, hog_params)

        for x_step in range(self.steps(scaled_width)):
            for y_step in range(self.steps(scaled_height)):
                y_pos = y_step * self.cell_per_steps
                x_pos = x_step * self.cell_per_steps
                x_left = x_pos * self.pixel_per_cell
                y_top = y_pos * self.pixel_per_cell

                x1, y1 = x_pos, y_pos
                x2, y2 = x1 + self.nblocks_per_window, y1 + self.nblocks_per_window
                hog_feat = []
                for ch in range(3):
                    hog_feat.append(hog[ch][y1:y2, x1:x2].ravel())
                hog_feats = np.hstack((hog_feat[0], hog_feat[1], hog_feat[2]))

                crop = roi[y_top:y_top + self.window, x_left:x_left + self.window]
                crop = cv2.resize(crop, (64, 64))
                spatial_features = spatial(crop, size=self.params.size)
                hist_features = color_histogram(crop, bins=self.params.nbins)

                stack = np.hstack((spatial_features,
                                   hist_features,
                                   hog_feats)).reshape(1, -1)
                x_data = X_scaler.transform(stack)
                is_car = svc.predict(x_data)

                if is_car:
                    window_scale = np.int(self.window * self.scale)

                    x1 = np.int(x_left * self.scale) + xstart
                    y1 = np.int(y_top * self.scale) + self.y_start
                    x2 = x1 + window_scale
                    y2 = y1 + window_scale

                    heatmap[y1:y2, x1:x2] += 1

        return heatmap


class Process(object):

    def __init__(self, clf, scaler, params):
        self.heatmap = []
        self.clf = clf
        self.scaler = scaler
        self.params = params

    def draw_box(self, img, labels):
        for car_number in range(1, labels[1]+1):
            non_zero = (labels[0] == car_number).nonzero()

            non_zero_y = np.array(non_zero[0])
            non_zero_x = np.array(non_zero[1])

            bbox = ((np.min(non_zero_x), np.min(non_zero_y)),
                    (np.max(non_zero_x), np.max(non_zero_y)))
            cv2.rectangle(img, bbox[0], bbox[1], (255, 0, 0), 2)
        return img

    def apply_threshold(self, heatmap):
        heatmap[heatmap <= self.params.heatmap_threshold] = 0
        return heatmap

    def process(self, img):
        heatmap1 = Heatmap(400, 700, 2, self.params)
        hm1 = heatmap1.process(img, self.clf, self.scaler)

        heatmap2 = Heatmap(400, 650, 1.5, self.params)
        hm2 = heatmap2.process(img, self.clf, self.scaler)

        heatmap3 = Heatmap(400, 500, 1, self.params)
        hm3 = heatmap3.process(img, self.clf, self.scaler)
        self.heatmap.append(hm1 + hm2 + hm3)

        labels = label(self.apply_threshold(sum(self.heatmap)))
        return self.draw_box(np.copy(img), labels)
