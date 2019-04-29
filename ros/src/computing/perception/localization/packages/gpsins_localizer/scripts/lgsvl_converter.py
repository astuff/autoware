#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Imu
from nmea_msgs.msg import Sentence
from novatel_gps_msgs.msg import Inspva
from nav_msgs.msg import Odometry

gpgga = None
velocity = None

def odom_cb(data):
    global velocity
    velocity = float(data.twist.twist.linear.x)

def nmea_cb(data):
    global inspva_pub
    global gpgga
    global velocity

    nmea_data = data.sentence.split(",")

    if nmea_data[0] == '$GPGGA':
        gpgga = nmea_data
        return
    elif nmea_data[0] == 'QQ02C':
        qq02c = nmea_data
    else:
        return

    if gpgga is None:
        print("No gpgga")
        return
    if velocity is None:
        print("No velocity")
        return

    # Combine GPGGA and QQ02C
    msg = Inspva();
    msg.header.stamp = data.header.stamp
    msg.status = "INS_SOLUTION_GOOD"
    msg.latitude = float(gpgga[2])
    msg.longitude = float(gpgga[4])

    # NOTE: this is wrong, we are using altitude instead of height because the
    # LG simulator doesn't give us height
    msg.height = float(gpgga[9])
    msg.roll = float(qq02c[4])
    msg.pitch = float(qq02c[5])
    msg.azimuth = float(qq02c[6])
    msg.north_velocity = velocity
    msg.east_velocity = 0.0
    inspva_pub.publish(msg)

def converter():

    global inspva_pub
    inspva_pub = rospy.Publisher("/gps/inspva", Inspva, queue_size=10)
    rospy.Subscriber("/nmea_sentence", Sentence, nmea_cb)
    rospy.Subscriber("/odom", Odometry, odom_cb)

    rospy.spin()

# Main function.
if __name__ == '__main__':

    # Initialize the node and name it.
    rospy.init_node('lgsvl_converter').

    converter()
