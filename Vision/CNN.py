# Copyright 2015 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""A very simple MNIST classifier.
See extensive documentation at
https://www.tensorflow.org/get_started/mnist/beginners
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

import tensorflow as tf

import h5py

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import random
import time

FLAGS = None


def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)


def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)


def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')


def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                          strides=[1, 2, 2, 1], padding='SAME')


def sample_batch(data, label, batchSize):
    idx = list(range(data.shape[0]))
    idxBatch = random.sample(idx, batchSize)
    #    for i in range(batchSize):
    #        imgplot = plt.imshow(data[idxBatch[i],:,:])
    #        plt.show()
    #        print(idxBatch[i])
    return (data[idxBatch, :, :],
            label[idxBatch, :])


def next_batch(data, label, i, batchSize):
    #    print(i, batchSize)
    return (data[int(i * batchSize):int((i + 1) * batchSize), :, :],
            label[int(i * batchSize):int((i + 1) * batchSize), :])


# def main(_):
# Import data
#  mnist = input_data.read_data_sets(FLAGS.data_dir, one_hot=True)

# load my gesture dataset
data = h5py.File('data128.mat')
data2 = h5py.File('dataTest128.mat')
print(data['imgTrain'].shape)
print(data['label'].shape)
dataSize = data['imgTrain'].shape[0]

batchSize = 120
numRound = 200
trainIter = int(dataSize / batchSize * numRound)
print("Total number of training iteration: %d" % trainIter)

batchSizeTest = batchSize

# Create the model


meanAccuracy = 0


dataTrain = np.squeeze(data['imgTrain'][:, :, :, :])
labelTrain = np.squeeze(data['label'][:, :])
dataTest = np.squeeze(data2['imgTest'][:, :, :, :])
labelTest = np.squeeze(data2['labelTest'][:, :])

dataTrain = np.transpose(dataTrain, (0, 3, 2, 1))
# labelTrain = np.transpose(labelTrain, (1, 0))
dataTest = np.transpose(dataTest, (0, 3, 2, 1))
# labelTest = np.transpose(labelTest, (1, 0))

#    orderTrain = np.reshape(np.mod(np.random.permutation(dataTrain.shape[0]*100), dataTrain.shape[0]), [-1, batchSize]);
#    print(orderTrain.shape)

x = tf.placeholder(tf.float32, [None, 128, 128, 3])

# Define loss and optimizer
y_ = tf.placeholder(tf.float32, [None, 12])

# CNN
# x_image = tf.reshape(x, [-1, 128, 128, 3])

# layer one
W_conv11 = weight_variable([5, 5, 3, 64])
b_conv11 = bias_variable([64])
h_conv11 = tf.nn.relu(conv2d(x, W_conv11) + b_conv11)

h_pool1 = max_pool_2x2(h_conv11)

#    # batch normalization, maybe wrong
#    axis = list(range(len(h_pool1.get_shape()) - 1))
#    mean, variance = tf.nn.moments(h_pool1, axis)
#    h_bn1 = tf.nn.batch_normalization(h_pool1, mean, variance, 0, 1, 1e-4)

# layer two
W_conv21 = weight_variable([5, 5, 64, 64])
b_conv21 = bias_variable([64])
h_conv21 = tf.nn.relu(conv2d(h_pool1, W_conv21) + b_conv21)

h_pool2 = max_pool_2x2(h_conv21)

#    # batch normalization, maybe wrong
#    axis = list(range(len(h_pool2.get_shape()) - 1))
#    mean, variance = tf.nn.moments(h_pool2, axis)
#    h_bn2 = tf.nn.batch_normalization(h_pool2, mean, variance, 0, 1, 1e-4)

# layer three
W_conv31 = weight_variable([5, 5, 64, 128])
b_conv31 = bias_variable([128])
h_conv31 = tf.nn.relu(conv2d(h_pool2, W_conv31) + b_conv31)

h_pool3 = max_pool_2x2(h_conv31)

# layer 4
W_conv41 = weight_variable([5, 5, 128, 128])
b_conv41 = bias_variable([128])
h_conv41 = tf.nn.relu(conv2d(h_pool3, W_conv41) + b_conv41)

h_pool4 = max_pool_2x2(h_conv41)

#    # batch normalization, maybe wrong
#    axis = list(range(len(h_pool3.get_shape()) - 1))
#    mean, variance = tf.nn.moments(h_pool3, axis)
#    h_bn3 = tf.nn.batch_normalization(h_pool3, mean, variance, 0, 1, 1e-4)

W_fc1 = weight_variable([8 * 8 * 128, 2048])
b_fc1 = bias_variable([2048])
h_pool3_flat = tf.reshape(h_pool4, [-1, 8 * 8 * 128])
h_fc1 = tf.nn.relu(tf.matmul(h_pool3_flat, W_fc1) + b_fc1)

keep_prob = tf.placeholder(tf.float32)
h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

W_fc2 = weight_variable([2048, 2048])
b_fc2 = bias_variable([2048])
h_fc2 = tf.nn.relu(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

h_fc2_drop = tf.nn.dropout(h_fc2, keep_prob)

W_fc3 = weight_variable([2048, 12])
b_fc3 = bias_variable([12])
y_conv = tf.matmul(h_fc2_drop, W_fc3) + b_fc3

cross_entropy = tf.reduce_mean(
    tf.nn.softmax_cross_entropy_with_logits_v2(labels=y_, logits=y_conv))

train_step = tf.train.AdamOptimizer(1e-4).minimize(cross_entropy)

correct_prediction = tf.equal(tf.argmax(y_conv, 1), tf.argmax(y_, 1))

accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

sess = tf.InteractiveSession()

# Add an op to initialize the variables.
init_op = tf.global_variables_initializer()

# Add ops to save and restore all the variables.
saver = tf.train.Saver(max_to_keep=4, keep_checkpoint_every_n_hours=2)

sess.run(init_op)
saver.save(sess, "E:/works/Topics/CNN_Robot/model/model_CNN", global_step=1000)

start_time = time.time()
for i in range(trainIter):
    batch = sample_batch(dataTrain, labelTrain, batchSize)
    # idx = i % (dataSize / batchSize)
    # batch = next_batch(dataTrain, labelTrain, idx, batchSize)
    train_step.run(feed_dict={x: batch[0], y_: batch[1], keep_prob: 0.5})
    if i % (dataSize / batchSize) == 0:
        train_accuracy = accuracy.eval(feed_dict={
            x: batch[0], y_: batch[1], keep_prob: 1.0})
        print("step %d, training accuracy %g" % (i, train_accuracy))
        duration = time.time() - start_time
        print(duration)
    if i % (dataSize / batchSize * 5) == 0:
        accuracyTest = 0
        for ii in range(int(dataTest.shape[0] / batchSizeTest)):
            accuracyTest += accuracy.eval(feed_dict={
                x: dataTest[ii * batchSizeTest: (ii + 1) * batchSizeTest - 1, :, :],
                y_: labelTest[ii * batchSizeTest: (ii + 1) * batchSizeTest - 1, :], keep_prob: 1.0})
        accuracyTest /= dataTest.shape[0] / batchSizeTest
        print("step %d, test accuracy Now %g" % (i, accuracyTest))
        saver.save(sess, "E:/works/Topics/CNN_Robot/model/model_CNN")




# accuracyNow = 0
# for i in range(int(dataTest.shape[0] / batchSizeTest)):
#     accuracyNow += accuracy.eval(feed_dict={
#         x: dataTest[i * batchSizeTest: (i + 1) * batchSizeTest - 1, :, :],
#         y_: labelTest[i * batchSizeTest: (i + 1) * batchSizeTest - 1, :], keep_prob: 1.0})
# accuracyNow /= dataTest.shape[0] / batchSizeTest
#
# meanAccuracy += accuracyNow
# print("**************** LOO CV no. %d, test accuracy %g *********************" % (iCV, accuracyNow))

tf.reset_default_graph()
sess.close()


