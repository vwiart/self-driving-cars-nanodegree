# Vehicle Detection
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## Histogram of Gradient (HOG)

The HOG is computed in the `hog_features` method in `classifier.py`. This method loops over each layer of the image and computes the HOG using the `sklearn` library.

The `Params` class contains the different parameters used to compute the HOG : orientations, pixels per cell, cells per block etc...

I tried different variation of those parameters and choose the following configuration : 
```
{
    'orientations': 9,
    'pixels_per_cell': (8, 8),
    'cells_per_block': (2, 2),
    'transform_sqrt': False,
    'visualise': False,
    'feature_vector': True,
}
```

I found that the `YCrCb` color space gave the best results during my tests.

Here's a representation of the HOG applied to an image on one channel : 

|image|HOG|
|-----|----------------------------|
|![cropped_black_car]|![hog]|

## Color spaces

The `color_histogram` and `spatial` methods extract the features of a given image with respect to the color space. 

|image|histogram of HSV color space|
|-----|----------------------------|
|![cropped_black_car]|![color_space]|

## Car classifier

The classifier is implemented in the `classifier.py` file.

The `extract_features` method take a list of file names and a set of parameters. It uses the `spatial`, `color_histogram` and `hog_features` methods to extract the different features from each image. The features are then concatenated and returned to the upstream method.

The `train` method uses the feactures extracted in `extract_features` to train a `LinearSVC` classifier. The features are first normalized using the `StandardScaler` class from `sklearn`

After the training phase, my model had a 98.789% accuracy

Here's an example of the classifier applied to two distinct area of an image (in red, the classifier predict the window was not a car; in green the window has been detected as a car)

![classification]

## Detect cars on an image

The `Process` class process an image in order to detect any car on them. The `process` method creates three heatmap, handling different level of details in the image.

For each heatmap, we process the image using the following pipeline (`process` method of the `Heatmap` class) :

1. A blank heatmap is created
2. The original image is cropped to select a smaller region of interest (the bottom half of the image)
3. The cropped image is converted to the selected color space (in our case YCrCb)
4. The HOG is computed for the whole cropped image. We want to avoid computing the HOG for each window because it is computationaly expensive.
5. The sliding window search is performed, for each window, we extract the features using the same exact pipeline used to train the classifier and apply those features to the classifier
6. If a car is detected, each pixel of the corresponding window is "incremented" to generate a heatmap.

Each heatmap is summed and we apply a threshold to eliminate **false positive**. The `label` method from `scipy` is used to identify the remaining heatmap and to draw a box (`draw_box` method of the `Process` class).

The video `project_video.mp4` located in the `output_images` is a video of the pipeline applied to the project video. [project_video.mp4](https://github.com/vwiart/self-driving-car-nanodegree-project5/blob/master/output_images/project_video.mp4)

## Discussion

This project was fun to implement. The classification part was quite easy to implement and I quickly managed to have more than 98% of accuracy without overfitting.

While implementing a sliding window search was not too difficult, I experienced difficulties to apply the classifier to each window, having sometime really weird result (eg. everything classified as a car). I believe my initial implementations were flawed by multiple minor mistakes (I'm a bit uncomfortable with non typed language).

This last implementation has been rewritten from scratch using the previous failure to improve the code base.

[//]: # (Image References)
[draw_box]: ./output_images/draw_box.jpg "Drawing a box"
[cropped_black_car]: ./output_images/cropped_black_car.jpg "Black car"
[color_space]: ./output_images/color_space.jpg "Color space features"
[hog]: ./output_images/hog.jpg "HOG"
[classification]: ./output_images/classification.png "Classification"