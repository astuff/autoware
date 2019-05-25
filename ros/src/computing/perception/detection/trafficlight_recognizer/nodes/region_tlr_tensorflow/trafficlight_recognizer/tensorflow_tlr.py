#!/usr/bin/env python

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from trafficlight_recognizer.srv import *
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
import tensorflow as tf
from keras.models import load_model
from keras.preprocessing.image import img_to_array

class RegionTlrTensorFlow:
    def __init__(self):
        self.USE_ALT_COLORSPACE = False
        self.TARGET_SIZE = (32, 32)
        self.NUM_CHANNELS = 3
        self.CLASSIFIER_STATE_MAP = {0:0, 1:3, 2:2, 3:1} # map from trained NN ordering to Autoware ordering

        self.cv_bridge = CvBridge()

    def preprocess_image(self, img):
        if self.USE_ALT_COLORSPACE:
            out_img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        else:
            out_img = img.copy()
    
        out_img = cv2.resize(out_img, self.TARGET_SIZE, interpolation=cv2.INTER_LINEAR)
        out_img = out_img.astype("float") / 255.0 # normalize to [0, 1]
        out_img = img_to_array(out_img)
        out_img = np.expand_dims(out_img, axis=0) # add dimension for batch index
    
        return out_img
    
    def recognize_light_state(self, req):
        # Prepare the input image then feed it into the NN for prediction
        cv_image = self.cv_bridge.imgmsg_to_cv2(req.roi_image, "passthrough")
        img = self.preprocess_image(cv_image)
    
        # Make the class prediction for this image and get a confidence value
        proba = self.nn_model.predict(img)
        confidence = np.amax(proba)
        class_id = self.CLASSIFIER_STATE_MAP[np.argmax(proba)]
        return [class_id, confidence]

    def run(self):
        rospy.init_node('tensorflow_tlr')
        
        # Workaround for cuDNN issue
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        session = tf.Session(config=config)
        
        # Setup the neural network
        self.nn_model_path = rospy.get_param('~nn_model_path')
        self.nn_model = load_model(self.nn_model_path)
        
        # Have to do this or the prediction in the callback fails
        proba = self.nn_model.predict(np.zeros((1, self.TARGET_SIZE[0], self.TARGET_SIZE[1], self.NUM_CHANNELS)))
        
        # Setup service
        self.service = rospy.Service('recognize_light_state', RecognizeLightState, self.recognize_light_state)
        
        rospy.spin()
        
if __name__ == "__main__":
    region_tlr_tensorflow = RegionTlrTensorFlow()
    region_tlr_tensorflow.run()
