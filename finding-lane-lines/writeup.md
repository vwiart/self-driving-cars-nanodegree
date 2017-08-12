# **Finding Lane Lines on the Road** 

## Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

During this project, I tried to follow the pipeline presented in the videos and iterate over it to try and understand the underlying logic.

The pipeline is located inside the `pipeline(initial)` method. My idea was to have a method that could apply the logic on individual image as well as on video (a video being basically a sequence of image, I would simply have to iterate over each image and to apply the `pipeline` method).

I've created a `vertices(image)` helper method to compute the vertices used in the `region_of_interest(img, vertices)` method

#### Hyper-parameters

I've set up hyper-parameters as global variables. I could have encapsulated them inside some class but it seemed overengineered at this point. The idea is to set up those parameters once and tune them once the algorithm is set up. Thus I've been able to improve the result

#### 1. Grayscale

I first transform the image by applying a grayscale filter. At this point, the colors does not matter so averaging each pixel on a gray scale seemed like a good option. A future improvement could be to make a distinction between permanent line with temporary one to make sure the algorithm select the right line (eg. when there are road maintenance)

#### 2. Gaussian blur
I apply a gaussian blur on the image in order to reduce the amount of noise in the image

#### 3. Canny
I apply the canny algorithm on the image in order to focus on the edges only.

#### 4. Region of interest
I crop off the part of image I'm not intereted in, ie. the parts that are not immediately in front of the camera. In my first iteration, I applied this method in the beginning of the pipeline ; my reasonning then was that it seemed a good idea to reduce the number of elements on the image earlier in the pipeline. But it resulted in some artefact on the image when applying the Canny algorithm (it would detect the edges of the cropped area). I decided to apply this part immediately after the Canny step.

#### 5. Hough_line & draw_line
This step is responsible of finding the different lines and to draw them. This step was the one on which I iterated the most to find the good hyper-parameters. Retrospectively, I should have used a more "scientific" approach to understand the hyper-parameters ; I feel that it just work "by chance" at the moment. I will try to understand more the underlying logic in it (I'm still not sure how rho and theta parameters works at the moment)

#### 6. Weighted image
This step simply superpose the initial image with the new one with lines

### 2. Identify potential shortcomings with your current pipeline

The draw lines method apply a linear regression to draw the lines. That means that the images with curves render poorly. Because of the incoming deadling, I'm not able to iterate over this but I would like to try more sophisticated regression to render the lines.

The images we used are really clean and it seems that my implementation works well on those but on the "optional challenge", I can see that there is room for improvement : reduce noises (tire marks), curves, shadows, artefacts...

### 3. Suggest possible improvements to your pipeline
1. Improve the cleanup of the image : get rid of the artefacts (shadows, ...)
2. Improve the regression to handle curves
3. Handle temporary lines to carefully select the lines when there is road maintenance for instance