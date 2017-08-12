import os

import numpy as np

from keras.layers import (
    Conv2D,
    Cropping2D,
    Dense, 
    Dropout,
    Flatten,
    Lambda)
from keras.layers.normalization import BatchNormalization
from keras.layers.pooling import MaxPooling2D
from keras.models import Sequential
from keras.optimizers import Adam
from keras.utils import plot_model

from sklearn.model_selection import train_test_split
import tensorflow as tf

import preprocessing

MODEL_FILENAME = 'model.h5'
PARAMS = {
    'learning_rate': 0.001,
    'dropout': 0.2,
    'epochs': 5,
    'batch_size': 64,
    'steering_correction': 0.10,
}


def _normalize(x):
    return (x - 128.0) / 128.0


def train():
    """ Preprocess the data and train the model."""
    model = Sequential()
    # Pre-process
    model.add(Cropping2D(cropping=((70, 25), (0, 0)),
                         input_shape=(160,320,3)))
    model.add(Lambda(_normalize))
    # Model
    model.add(Conv2D(24, (5, 5), strides=(2, 2), activation='relu'))
    model.add(BatchNormalization())
    model.add(Conv2D(36, (5, 5), strides=(2, 2), activation='relu'))
    model.add(BatchNormalization())
    model.add(Conv2D(48, (5, 5), strides=(2, 2), activation='relu'))
    model.add(BatchNormalization())
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(BatchNormalization())
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(BatchNormalization())
    model.add(Flatten())
    model.add(Dense(100))
    model.add(BatchNormalization())
    model.add(Dropout(PARAMS['dropout']))
    model.add(Dense(50))
    model.add(BatchNormalization())
    model.add(Dropout(PARAMS['dropout']))
    model.add(Dense(10))
    model.add(Dense(1))

    # Train
    model.compile(loss='mse', optimizer=Adam(lr=PARAMS['learning_rate']))

    data = preprocessing.parse('data/')
    training_set, validation_set = train_test_split(data, test_size=0.2)
    x_train = preprocessing.batches('data/',
                                    training_set,
                                    PARAMS['batch_size'],
                                    PARAMS['steering_correction'])
    x_valid = preprocessing.batches('data/',
                                    validation_set,
                                    PARAMS['batch_size'],
                                    PARAMS['steering_correction'])

    steps_per_epoch = len(training_set) / PARAMS['batch_size']
    validation_steps = len(validation_set) / PARAMS['batch_size']

    model.fit_generator(x_train,
                        steps_per_epoch=steps_per_epoch,
                        validation_data=x_valid,
                        validation_steps=validation_steps,
                        epochs=PARAMS['epochs'])
    model.summary()
    plot_model(model, to_file='article/model.png')
    # model.save(MODEL_FILENAME) 


if __name__ == '__main__':
    train()

    