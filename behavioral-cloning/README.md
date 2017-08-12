# Behavioral Cloning

The project include the following files :
* `model.py` : contains the definition of the model and the methods to train it
* `preprocessing.py` : contains the methods to load data from the hard disk and pre-process it to the model
* `model.h5` : an export of the trained model
* `video.mp4` : a video from the simulator in autonomous mode

The goal of the project is to train a model to allow a car to run in a simulated environment :
* Use the simulator to extract images and steering values to train a model
* Pre-process the data in order to facilitate the training of the neural network
* Train and validate the model
* Use the model to drive the car in autonomous mode

## How-to use the project

1. Generate some training data using the simulator and put the data in the `./data/` folder
2. Train the model using `python model.py`
3. Run the simulator in autonomous mode
4. Run the client using `python drive.py`
5. Enjoy ! :)

## Pre-processing

The pre-processing step is located in the `preprocessing.py` module. It contains 3 mains methods :
* `load_image` : loads an image from the hard disk
* `batches` : Divide the dataset in smaller batch to facilitate the training
* `parse` : parse the csv file

In order to limit the bias toward the shape of the track, a flip operation is processed ; that allows the model to train on an equal number of left and right curve.

## Model

The model that has been implemented is the model presented in the [nvidia blog](https://devblogs.nvidia.com/parallelforall/deep-learning-self-driving-cars/)

![Nvidia model][model]

I first tried to use a LeNet model but the first results where not satisfying. The model presented in the Nvidia article seems to fit the project well and gave good results.

The final model is the following :

|Layer|Output shape|Comment|
|-----|------------|-------|
|Cropping|65x320x3 |Crop the image to remove the unimportant part (1)|
|Lambda|65x320x3 |Normalize the image|
|Convolutional layer #1|31x158x24|First convolutional layer with a 5x5 kernel and 2x2 strides|
|Batch normalization #1|31x158x24|(2)|
|Convolutional layer #2|14x77x36|Second convolutional layer with a 5x5 kernel and 2x2 strides|
|Batch normalization #2|14x77x36|(2)|
|Convolutional layer #3|5x37x48|Third convolutional layer with a 5x5 kernel and 2x2 strides|
|Batch normalization #3|5x37x48|(2)|
|Convolutional layer #4|3x35x64|Fourth convolutional layer with a 3x3 kernel|
|Batch normalization #4|3x35x64|(2)|
|Convolutional layer #5|3x33x64|Fifth convolutional layer with a 3x3 kernel|
|Batch normalization #5|3x33x64|(2)|
|Convolutional layer #6|1x33x64|Sixth convolutional layer with a 3x3 kernel|
|Batch normalization #6|1x33x64|(2)|
|Flatten layer|2112|Flatten the output to a vector|
|Fully connected layer #1|100| First fully connected layer|
|Batch normalization #7|100|(2)|
|Dropout #1|100| (3)|
|Fully connected layer #2|50| Second fully connected layer|
|Batch normalization #7|50|(2)|
|Dropout #2|50| (3)|
|Fully connected layer #3|10| Third fully connected layer|
|Fully connected layer #4|2| Third fully connected layer|

(1) The images have been cropped to remove unnecessary part from it. It allows the model to focus on the most important features.

![Original image][original_image]
![Cropped image][cropped_image]

(2) Batch normalization grants faster learning and a better accuracy by normalizing the different batches. [ref](https://www.quora.com/Why-does-batch-normalization-help)

(3) A dropout layer has been introduced to reduce overfitting by randomly disabling some neuron [ref](https://www.quora.com/How-does-the-dropout-method-work-in-deep-learning)

## Training

The model has been trained using those hyper-parameters:
* `learning rate`: 0.001
* `dropout`: 0.2
* `epochs`: 5
* `batch size`: 64
* `steering correction`: 0.10

The dataset has been split in 80% test data and 20% validation data to measure overfitting. A dropout of 0.2 has been applied after the first two Fully Connected layers to prevent overfitting as well.

The model has been first trained on a dataset captured while making 2 laps on the track, trying to keep the vehicle in the center of the track. It has then been tuned by applying the following process:
1. Run the model in the simulator
2. Identify issue in the driving (eg. crossing border)
3. Use the simulator to train new data to prevent the issue to happen (eg. avoid the border)
4. Train the model using the previous weights (transfer learning)
5. Goto 1

Here's a visualisation of the repartition of the steering angle used for the first training dataset

![Dataset][data_distrib]

The dataset has been altered during the process due to the multiple refine steps while training the dataset on specific patterns (eg. avoiding a border). Unfortunately, I overwrote the different data during each iteration so I'm unable to display the real data distribution used. By training the dataset on specific patterns, the distribution must have been balanced since I mostly worked on avoiding obstacle.

In retrospect, I should have reduce the data imbalance (too many data with no steering) : this would have allowed a better initial result. 

After multiple try and refine, the model has been able to perform on the track.


[//]: # (Image References)
[model]: ./article/cnn-architecture-624x890.png "Model"
[original_image]: ./article/original_image.jpg "Original image"
[cropped_image]: ./article/cropped_image.jpg "Cropped image"
[original_data]: ./article/orig_data.png "Original data"
[flip_data]: ./article/flip_data.png "Flipped data"
[data_distrib]: ./article/data_distrib.jpg "Flipped data"