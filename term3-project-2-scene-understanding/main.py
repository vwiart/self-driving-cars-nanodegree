import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests

EPOCHS=10
BATCH_SIZE=8
KEEP_PROB = 0.9
LEARNING_RATE = 0.0001


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()
    input_ = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3 = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4 = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7 = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    
    return input_, keep, layer3, layer4, layer7
tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    initializer = tf.truncated_normal_initializer(stddev=0.01)
    regularizer = tf.contrib.layers.l2_regularizer(1e-3)
    conv1 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, strides=(1, 1),
                             padding='SAME',
                             kernel_initializer=initializer,
                             kernel_regularizer=regularizer)
    conv2 = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, strides=(1, 1),
                             padding='SAME',
                             kernel_initializer=initializer,
                             kernel_regularizer=regularizer)
    conv3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, strides=(1, 1),
                             padding='SAME',
                             kernel_initializer=initializer,
                             kernel_regularizer=regularizer)
    out1 = tf.layers.conv2d_transpose(conv1, num_classes, 4, strides=(2, 2),
                                      padding='SAME',
                                      kernel_initializer=initializer,
                                      kernel_regularizer=regularizer)
    out1 = tf.add(out1, conv2)
    
    out2 = tf.layers.conv2d_transpose(out1, num_classes, 4, strides=(2, 2),
                                      padding='SAME',
                                      kernel_initializer=initializer,
                                      kernel_regularizer=regularizer)
    out2 = tf.add(out2, conv3)

    out = tf.layers.conv2d_transpose(out2, num_classes, 16, strides=(8, 8),
                                     padding= 'SAME',
                                     kernel_initializer=initializer,
                                     kernel_regularizer=regularizer)
    
    return out
tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    correct_label = tf.reshape(correct_label, (-1, num_classes))

    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(labels=correct_label,
                                                                                logits = logits))
    reg_losses = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES)
    cross_entropy_loss += sum(reg_losses)

    train_op = tf.train.AdamOptimizer(learning_rate).minimize(cross_entropy_loss)
    return logits, train_op, cross_entropy_loss
tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    sess.run(tf.global_variables_initializer())
    for e in range(epochs):
        print("Epoch {epoch}".format(epoch=e))
        for image, label in get_batches_fn(batch_size):
            fd = {
                input_image: image,
                correct_label: label,
                keep_prob: KEEP_PROB,
                learning_rate: LEARNING_RATE
            }
            _, loss = sess.run([train_op, cross_entropy_loss], feed_dict=fd)
            print("loss: {:.3f}".format(loss))
tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    epochs = EPOCHS
    batch_size = BATCH_SIZE

    with tf.Session() as sess:
        vgg_path = os.path.join(data_dir, 'vgg')
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'),
                                                   image_shape)
        correct_label = tf.placeholder(tf.float32,
                                       shape=[None, None, None, num_classes])
        learning_rate = tf.placeholder(tf.float32)
        l1, keep, l3, l4, l7 = load_vgg(sess, vgg_path)
        out = layers(l3, l4, l7, num_classes)
        logits, train_op, cross_entropy_loss = optimize(out, correct_label,
                                                        learning_rate,
                                                        num_classes)
        train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, l1,
                 correct_label, keep, learning_rate)
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape,
                                     logits, keep, l1)

if __name__ == '__main__':
    run()
