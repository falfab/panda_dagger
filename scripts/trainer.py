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

print ("create session")
sess = tf.InteractiveSession()

print ("create agent")
model = agent.Agent(name='model_trained', sess=sess)

# carica dataset da convertire in np
os.chdir(os.path.dirname(os.path.realpath(__file__)))

print("load data")
data = []
with open('../dataset/merged.pkl', 'rb') as fd:
    data = pickle.load(fd)

print  "concatenate",len(data),"image" 
image_all  = np.zeros( (len(data),agent.img_dim[0], agent.img_dim[1], agent.img_dim[2]))
action_all = np.zeros( (len(data),agent.n_action) )
bridge = cv_bridge.CvBridge()
for index, d in enumerate(data):
    if index % 1000 == 0:
        print "concatenated",index,"/",len(data) 

    mat = bridge.imgmsg_to_cv2(d[0], desired_encoding='passthrough')
    action = [d[1][0],d[1][1],d[1][2]]
    image_all[index] = mat
    action_all[index] = action

del data

# train
print ('start training')
model.train(image_all, action_all, print_freq=5)

# salva il modello
model.save_model()

