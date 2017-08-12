# Traffic Sign Recognition


**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report

The project is hosted on [github](https://github.com/vwiart/self-driving-car-nanodegree-project2)

## 1. Dataset exploration

The initial dataset is composed of 51 839 images, shuffled as followed :
 - 34 799 in the training dataset
 - 12 630 in the testing dataset
 - 4 410 in the validation dataset

Each image has a shape of 32x32x3. There is 43 different classes.

![List of all classes][all_classes]

The following diagram presents the distribution for each classes : 

![Class distribution][class_distribution]

As you can see, the images are not evenly distributed in the dataset. There's plenty of images for the `Speed limit (30km/h)` class or `Speed limit (50km/h)` class and few images for the `Speed limit (20km/h)` and `Dangerous curve to the left`.

In order to balance this distribution, I decided to generate some new images applying random transformation (rotation, translation) to the existing dataset. After this operation, the training dataset has been pushed to 86 430 images.

## 2. Architecture of the solution

The pipeline is based on the LeNet model :
1. the data is preprocessed to enhance the quality
2. The data goes through the LeNet model to train it
3. Each 5 epochs, the model is validated against the validation dataset
4. After training, the model is tested against the test dataset
5. The model is tested against random images found on the internet

### Pre-processing

The training dataset consists of 34 799 32x32x3 images. I've extracted some statistics of this dataset: 
 - The minimal value if 0
 - The maximal value is 255
 - The mean value is 82

Running this data through the pipeline would make the model harder to train due to the data distribution. The model would be easier to train with data distributed in [-1, 1] instead of [0, 255]. In order to facilitate training, a pre-processing step has been applied :
 1. Apply a grayscale filter to transform the 32x32x3 image to a 32x32x1 image
 2. Enhance the contrast ([Contrast Limited AHE](https://en.wikipedia.org/wiki/Adaptive_histogram_equalization))
 3. Normalisation : (cf. the `def normalize(img)` method)

In order to balance the class distribution, a rotation and a translation transformation have been applied to some images. 

Although the colour is an important data in the dataset (eg. to distinguish between permanent and temporary signs), I choose to ignore it and focus on the shape detection by applying the grayscale filter.

I've enhanced the contrast to make the shapes more sharp. The normalisation is applied to make the pixel distribution more even and center around 0.

Here's an example of the pre-processing step, including a rotation + translation operation

![Initial image][initial_image]
![Processed image][transformed_image]

### Model

I used the LeNet model.

**Convolution layer #1**
The input is a 32x32x1 matrix. I apply a 5x5 kernel and a 1x1 stride on the convolutional part plus a max pooling with a 2x2 kernel and 2x2 stride. It outputs a 14x14x6 matrix

**Convolution layer #2**
The input is a 14x14x6 matrix. I apply a 5x5 kernel and a 1x1 stride on the convolutional part plus a max pooling with a 2x2 kernel and 2x2 stride. It outputs a 5x5x16 matrix

**Flatten layer**
The input is a 5x5x16 matrix. It flattens the matrix to a 1x400 vector

**Fully connected layer #1**
This layer takes the flattened layer output and make the data goes through a fully connected layer. It applies a dropout to prevent the model to overfit. The output is a 1x120 matrix

**Fully connected layer #2**
This layer takes the FC#1 output and make the data goes through a fully connected layer. It applies a dropout to prevent the model to overfit. The output is a 1x84 matrix

**Fully connected layer #3**
This layer takes the FC#2 output and make the data goes through a fully connected layer. It applies a dropout to prevent the model to overfit. The output is a 1x43 matrix corresponding to each class in the dataset

### Training & testing

The model has been trained on 25 epochs with a 0.001 learning rate.

Here's an extract of the last training :
```
Training ...
Epoch 0 - Validation Accuracy = 0.604
Epoch 5 - Validation Accuracy = 0.889
Epoch 10 - Validation Accuracy = 0.921
Epoch 15 - Validation Accuracy = 0.935
Epoch 20 - Validation Accuracy = 0.932
Epoch 24 - Validation Accuracy = 0.939
Done
```

The current accuracy is not great but still acceptable. Running the model with the test dataset has an accuracy of 0.934 which seems to indicate the model does not overfit and generalize well

## 3. Testing on a "real world" data

I've looked for traffic signs on google corresponding to different classes of the dataset. I've transformed (scaling, cropping) those image in order for them to match the input size (32x32x3):

| Classe               |Image            |
|----------------------|-----------------|
|Speed limit : 120 km/h|![image1][image1]|
|No passing            |![image2][image2]|
|Priority road         |![image3][image3]|
|General caution       |![image4][image4]|
|Slippery road         |![image5][image5]|

Each image was pre-processed using the same pipeline described in the `pre-processing` paragraph. The processed image was used as input in the model and produced those predictions :

![results][results]

4 out of 5 predictions were correct. The incorrect prediction was for the `slippery road` traffic sign. The model predicted `Right-of-way at the next intersection`. The model still recognized that it was a triangular sign :)

Looking at the softmax result, it seems that the model is almost always pretty sure of its prediction, having results between 80% and 100%. The wrong prediction has only a certainty of 70%.

I think the model would have better prediction if the image had a better resolution. 


[//]: # (Image References)
[all_classes]: ./writeup/all_classes.png "All classes"
[class_distribution]: ./writeup/class_distribution.png "Class distribution"
[initial_image]: ./writeup/preprocessing.png "Pre-processing"
[transformed_image]: ./writeup/preprocessing2.png "Pre-processing"
[image1]: ./signs/8.jpg "Speed limit 120"
[image2]: ./signs/9.jpg "No passing"
[image3]: ./signs/12.jpg "Priority road"
[image4]: ./signs/18.jpg "General caution"
[image5]: ./signs/23.jpg "Slippery road"
[results]: ./writeup/results.png "Slippery road"
