import tensorflow as tf
import tensorlayer as tl
from tensorlayer.layers import *
import time

img_dim = [64, 64, 3]
n_action = 3
batch_size = 32
n_epoch = 100

class Agent(object):
    def __init__(self, name='model', sess=None):
        assert sess != None
        self.name = name
        self.sess = sess

        self.x = tf.placeholder(tf.float32, [None, img_dim[0], img_dim[1], img_dim[2]], name='Observaion')
        self.y = tf.placeholder(tf.float32, [None, n_action], name='Steer')

        self._build_net(True, False)
        self._build_net(False, True)
        self._define_train_ops()

        tl.layers.initialize_global_variables(self.sess)

        print "-------- LAYERS ----------"
        self.n_test.print_layers()
        print "-------- PARAMS ----------"        
        self.n_test.print_params(False)
        print()
        # exit()

    def _build_net(self, is_train=True, reuse=None):
        with tf.variable_scope(self.name, reuse=reuse) as vs:
            #tl.layers.set_name_reuse(reuse)

            # TODO: set the same seed to DropoutLayer?
            # TODO: set the data_format to have same execution on CPU and GPU

            n = InputLayer(self.x / 255, name='in')

            n = Conv2d(n, 32, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c1/1')
            n = Conv2d(n, 32, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c1/2')
            n = MaxPool2d(n, (2, 2), (2, 2), 'VALID', name='max1')

            n = DropoutLayer(n, 0.75, is_fix=True, is_train=is_train, name='drop1')

            n = Conv2d(n, 64, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c2/1')
            n = Conv2d(n, 64, (3, 3), (1, 1), tf.nn.relu, "VALID", name='c2/2')
            n = MaxPool2d(n, (2, 2), (2, 2), 'VALID', name='max2')
            # print(n.outputs)
            n = DropoutLayer(n, 0.75, is_fix=True, is_train=is_train, name='drop2')

            n = FlattenLayer(n, name='f')
            n = DenseLayer(n, 512, tf.nn.relu, name='dense1')
            n = DropoutLayer(n, 0.5, is_fix=True, is_train=is_train, name='drop3')
            n = DenseLayer(n, n_action, tf.nn.tanh, name='o')

        if is_train:
            self.n_train = n
        else:
            self.n_test = n

    def _define_train_ops(self):
        self.cost = tl.cost.mean_squared_error(self.n_train.outputs, self.y, is_mean=False)
        self.train_params = tl.layers.get_variables_with_name(self.name, train_only=True, printable=False)
        self.train_op = tf.train.AdamOptimizer(learning_rate=0.0001).minimize(self.cost, var_list=self.train_params)

    def train(self, X, y, n_epoch=n_epoch, batch_size=batch_size, print_freq=20):
        for epoch in range(n_epoch):
            start_time = time.time()
            total_err, n_iter = 0, 0
            for X_, y_ in tl.iterate.minibatches(X, y, batch_size, shuffle=True):
                _, err = self.sess.run([self.train_op, self.cost], feed_dict={self.x: X_, self.y: y_})
                total_err += err
                n_iter += 1
            if epoch % print_freq == 0:
                print("Epoch [%d/%d] cost:%f took:%fs" % (epoch, n_epoch, total_err/n_iter, time.time()-start_time))

    def predict(self, image):
        a = self.sess.run(self.n_test.outputs, {self.x : image})
        return a

    def save_model(self):
        tl.files.save_npz(self.n_test.all_params, name=self.name, sess=self.sess)

    def load_model(self):
        tl.files.load_and_assign_npz(sess=self.sess, name=self.name, network=self.n_test)
