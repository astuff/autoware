#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from autoware_msgs.msg import TrafficLight

USE_ALT_COLORSPACE = False
TARGET_SIZE = (32, 32)
NUM_CHANNELS = 3
IDX_DEBOUNCE_THRESHOLD = 2
CONFIDENCE_THRESHOLD = 0.6
PUB_TIMEOUT = 10.0  # seconds
ROI_IMG_TIMEOUT = 1.0  # seconds
states = ['red', 'green', 'off']
# internal_state_map = {0:'green', 1:'off', 2:'red', 3:'yellow' }
classifier_state_map = {0: 1, 1: 2, 2: 0, 3: 0}  # note, treating yellow as red

classifier_state_strings = {0: 'red signal', 1: 'green signal', 2: ''}

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config=config)

cv_bridge = CvBridge()
idx_debounce_count = 0
last_pub_state_idx = -1
last_debounce_state_idx = -1
img_timer = None

result_msg = TrafficLight()


def preprocess_image(img):
    out_img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    return out_img


def ROI_image_callback(image_msg):
    global idx_debounce_count, \
        IDX_DEBOUNCE_THRESHOLD, \
        last_pub_state_idx, \
        last_debounce_state_idx, \
        last_pub_time, \
        img_timer

    if img_timer is not None:
        img_timer.shutdown()

    img_timer = rospy.Timer(rospy.Duration(ROI_IMG_TIMEOUT), ImgTimeoutCallback)
    last_ROI_img_time = rospy.get_rostime()

    # Resize the input image then feed it into the NN for prediction
    cv_image = cv_bridge.imgmsg_to_cv2(image_msg, "passthrough")

    if USE_ALT_COLORSPACE:
        cv_image = preprocess_image(cv_image)

    img = cv2.resize(cv_image, TARGET_SIZE, interpolation=cv2.INTER_LINEAR)
    img = img.astype("float") / 255.0
    img = img_to_array(img)
    img = np.expand_dims(img, axis=0)

    proba = nn_model.predict(img)
    confidence = np.amax(proba)
    predicted_state = nn_model.predict_classes(img)

    idx = -1
    if confidence >= CONFIDENCE_THRESHOLD:
        # Get the prediction
        idx = np.argmax(proba)
        idx = classifier_state_map[idx]

        # Debounce the prediction before publishing
        if idx == last_debounce_state_idx:
            idx_debounce_count += 1
            if idx_debounce_count > IDX_DEBOUNCE_THRESHOLD:
                idx_debounce_count = IDX_DEBOUNCE_THRESHOLD  # prevent overflow
        else:
            last_debounce_state_idx = idx
            idx_debounce_count = 0

        if idx_debounce_count >= IDX_DEBOUNCE_THRESHOLD:
            # Only consider publishing debounced states
            if last_debounce_state_idx != last_pub_state_idx or \
                    (rospy.get_rostime() - last_pub_time > rospy.Duration(PUB_TIMEOUT) and
                        last_pub_state_idx != 2):
                # Only publish when state changes or a timeout occurs (but only want to repeat non-"unknown" states)
                result_msg.traffic_light = last_debounce_state_idx
                light_color_pub.publish(result_msg)

                if last_debounce_state_idx != last_pub_state_idx:
                    light_color_string_pub.publish(classifier_state_strings[last_debounce_state_idx])

                last_pub_state_idx = last_debounce_state_idx
                last_pub_time = rospy.get_rostime()


def ImgTimeoutCallback(event):
    global last_pub_state_idx, last_pub_time
    if last_pub_state_idx != 2:
        result_msg.traffic_light = 2
        light_color_pub.publish(result_msg)
        light_color_string_pub.publish(classifier_state_strings[2])
        last_pub_state_idx = 2
        last_pub_time = rospy.get_rostime()

rospy.init_node('tlr_classifier')

nn_model_path = rospy.get_param('~nn_model_path')
nn_model = load_model(nn_model_path)

# Have to do this or the prediction in the callback fails
proba = nn_model.predict(np.zeros((1, TARGET_SIZE[0], TARGET_SIZE[1], NUM_CHANNELS)))

rospy.Subscriber('tlr_roi_image', Image, ROI_image_callback, queue_size=1)
light_color_pub = rospy.Publisher('light_color', TrafficLight, queue_size=1)
light_color_string_pub = rospy.Publisher('sound_player', String, queue_size=1)

last_pub_time = rospy.get_rostime()

rospy.spin()
