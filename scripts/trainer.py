import agent
import pickle
import os
import numpy as np
import tensorflow as tf
import cv_bridge

def img_reshape(input_img):
    #""" (3, 64, 64) --> (64, 64, 3) """
    #_img = np.transpose(input_img, (0, 1, 2))
    #_img = np.flipud(_img)
    _img = np.reshape(input_img,
            (1, agent.img_dim[0], agent.img_dim[1], agent.img_dim[2]))
    return _img

# crea agent
sess = tf.InteractiveSession()
model = agent.Agent(name='model_trained', sess=sess)

# carica dataset da convertire in np
os.chdir(os.path.dirname(os.path.realpath(__file__)))

data = []
with open('../dataset/merged.pkl', 'rb') as fd:
    data = pickle.load(fd)

image_all  = np.zeros( (0,agent.img_dim[0], agent.img_dim[1], agent.img_dim[2]))
action_all = np.zeros( (0,agent.n_action) )

bridge = cv_bridge.CvBridge()
for d in data:
    mat = bridge.imgmsg_to_cv2(d[0], desired_encoding='passthrough')
    action = [ d[1].x,d[1].y,d[1].z]

    image_all  = np.concatenate( [image_all , np.reshape(mat,(1, agent.img_dim[0], agent.img_dim[1], agent.img_dim[2])) ], axis=0 ) # TODO img_reshape???
    action_all = np.concatenate( [action_all, np.reshape(action, [1,agent.n_action]) ], axis=0 )

del data

# train
model.train(image_all, action_all)

# salva il modello
model.save_model()

