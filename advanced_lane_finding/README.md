# Advanced Lane Finding
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## I. Camera calibration

The camera calibration is done by using the `calibrateCamera` method from opencv.

The `calibrate` method in `camera.py` accepts a list of images as an argument. For each image, the corners are looked up via `findChessboardCorners`.

The openCV's `calibrateCamera` method is used to compute the `camera_matrix` and the `distorsion_coef` which are returned to the caller.

Here's an example of the original images and their undistorted counter part :

|Image|Undistorted|
|-----|-----------|
|![Calibration image 1][orig1]|![Undistorted image 1][undistorted1]|
|![Calibration image 2][orig2]|![Undistorted image 2][undistorted2]|
|![Calibration image 3][orig3]|![Undistorted image 3][undistorted3]|

## II. Perspective transform

The goal of this step is to transform the image from the camera point of view to a point located over the road (bird view).

I have defined a `Point` class that stores point coordinates (x, y). I have also defined a `Rectangle` class that stores 4 `Point` defining a `Rectangle`. The idea is to define a `Rectangle` in the source plane and another `Rectangle` is the destination plane and compute the transformation matrix to go from the source to the destination plane using the `getPerspectiveTransform` method :

```
def _perspective(img, src, dst):
    """transform the perspective using the source and destination.
    Args:
        src: A `Rectangle`
        dst: A `Rectangle`
    Returns:
        An image
    """
    M = cv2.getPerspectiveTransform(src.to_array(), dst.to_array())
    img_size = (img.shape[1], img.shape[0])
    return cv2.warpPerspective(img, M, img_size)
```

|Image|Bird view|
|-----|-----------|
|![Calibration image 2][orig2]|![Undistorted image 1][perspective_transform1]|
|![Calibration image 4][orig4]|![Undistorted image 2][perspective_transform2]|
|![Straight lines 1][orig5]|![Perspective transform 3][perspective_transform3]|

For the `Straight lines 1` image, I've added red lines to help visualize the rectangle points. Those lines are not in the source and rendered image

The transformation algorithm can be found in `perspective.py` :

- `test_calibration2`: perspective transform for the `calibration2.jpg`
- `test_calibration13`: perspective transform for the `calibration13.jpg`
- `test_st_lines1`: perspective transform for the `straight_lines1.jpg`

## III. Gradient threshold

The `_gradient` method from the image module transform the image into a black and white image.

We first extract the hsv color channel from the image. Only the `l` and `s` channel are going to be used in the algorithm :
 1. Apply the sobel operator to the `l` channel, take the absolute value, scale the result and apply lower and upper threshold
 2. Apply the sobel operator on the `s` channel and apply the lower and upper threshold
 3. Combine the (1) and (2)

|Image|Gradient threshold|
|-----|-----------|
|![Straight lines 1][orig6]|![Gradient transform 1][gradient_transform1]|
|![Test 1][orig7]|![Gradient transform 2][gradient_transform2]|
|![Test 5][orig8]|![Gradient transform 2][gradient_transform3]|

## IV. Pipeline

The pipeline is the list of transformation we apply to the image to find the lane lines

We use the 'test5.jpg' image to illustrate this process:

![Initial image][orig8]

### 1. Undistort
![Undistorted][pipeline_undistort]

### 2. Gradient threshold
![Gradient][pipeline_gradient]

### 3. Perspective transform
![Perspective transform][pipeline_perspective]

We can then plot the corresponding histogram : 

![Histogram][pipeline_histogram]

We can see two spikes on that histogram (one around 300, the other around 1000). Those are the root of the lane lines. We apply the sliding windows algorithm (`_sliding_windows` method) to find the lane lines

![Sliding windows][pipeline_sliding]

### 4. Compute curvature and offset

The `curvature` and `offset_from_center` compute the curvature of the lane lines and the position of the car on the road. 

The curvature is computed usigin the second order polymonials of each lane line : f(y) = `Ay² +By + C`.
From this formula, we compute the radius of the curvature using the following function : `compute = lambda x: ((1 + (2 * x[0] * np.max(ploty) * ym_ppx + x[1]) ** 2) ** 1.5) / np.absolute(2 * x[0])` where `x` is the polynomial of one lane line (eg. `x = [2, 3, 4]` if the polynomial is `f(y) = 2y² + 3y + 4`)

The offset is computed using the following `(img_center - lane_center) * xm_ppx` where `img_center` is the center of the image, `lane_center` is the average of the bias of the two lane lines.

The `add_text` method allows to display those information on the image

### 5. Draw lines

The lines are drawn using the `draw_line` method. We used the line defined by the sliding windows algorithm:

1. We draw on a blank image
2. We "unwarp" the image using the `warpPerspective` method with the inverse matrix used to do the perspective transform
3. We add the generated image to the undistorted image

## V. Video

A frame is read from the source video. It is transformed through the `pipeline` method and put into the output video.

## VI. Discussion

I had a lot of fun working on this project. Although the initial implementation was quickly built (A few hours and I had the pipeline set up), the end result was not great so I had to reiterate to improve the result.

The first issue I had to fix was the perspective transform : lines that should have been parallel in the birds view weren't parallel. I had to tweak a bit the parameters to make it more close to reality.

After the first submit of the project, the reviewer pointed out that some section were not properly handled by the pipeline. In order to fix those issues, I had to fix a lot of problems.

The first thing I did was to reimplement the whole project (cf. [commit #79a55a5](https://github.com/vwiart/self-driving-car-nanodegree-project4/commit/79a55a5b136576fe5daca50aad841573e4d8386e)): I went from a stateless to a stateful pipeline. The idea was to introduce some state to discard glitches in some frames. When a glitch is detected, the frame is skipped. One drawback of this approach is if too many glitches in a row could introduce weird behavior. An alternative approach would be to try to infer curvature based on the previous frame and the current frame.

I then fixed the `gradient threshold` part : the parameters have been tuned and I review the implementation:
```
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
```

This implementation fixed the issue especially on the yellow line detection. I've tried using the pipeline on harder challenge and I believe some improvement can be made on that part. I've noticed for instance that variation in luminosity can lead to some glitches. A more complex implementation would improve this issue.

[//]: # (Image References)
[orig1]: ./camera_cal/calibration1.jpg "Calibration image 1"
[orig2]: ./camera_cal/calibration2.jpg "Calibration image 2"
[orig3]: ./camera_cal/calibration17.jpg "Calibration image 3"
[orig4]: ./camera_cal/calibration13.jpg "Calibration image 4"
[orig5]: ./test_images/straight_lines1_with_view.jpg "Straight lines 1 with lines"
[orig6]: ./test_images/straight_lines1.jpg "Straight lines 1"
[orig7]: ./test_images/test1.jpg "Test 1"
[orig8]: ./test_images/test5.jpg "Test 5"
[undistorted1]: ./output_images/undistorted/calibration1.jpg "Undistorted image 1"
[undistorted2]: ./output_images/undistorted/calibration2.jpg "Undistorted image 2"
[undistorted3]: ./output_images/undistorted/calibration17.jpg "Undistorted image 3"
[perspective_transform1]: ./output_images/perspective_transform/calibration2.jpg "Perspective transform 1"
[perspective_transform2]: ./output_images/perspective_transform/calibration13.jpg "Perspective transform 2"
[perspective_transform3]: ./output_images/perspective_transform/straight_lines1.jpg "Perspective transform 3"
[gradient_transform1]: ./output_images/gradient/straight_lines1.png "Gradient transform 1"
[gradient_transform2]: ./output_images/gradient/test1.png "Gradient transform 2"
[gradient_transform3]: ./output_images/gradient/test5.png "Gradient transform 3"

[pipeline_undistort]: ./output_images/pipeline/undistorted.png "Undistorted image "
[pipeline_gradient]: ./output_images/pipeline/gradient.png "gradient"
[pipeline_perspective]: ./output_images/pipeline/perspective.png "gradient"
[pipeline_histogram]: ./output_images/pipeline/histogram.png "histogram"
[pipeline_sliding]: ./output_images/pipeline/sliding_windows.png "sliding windows"
