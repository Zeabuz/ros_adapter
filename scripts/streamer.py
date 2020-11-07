#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import grpc
import cv2

import sensordata_pb2
import sensordata_pb2_grpc

import numpy as np

def streamer():

    # Setting up cv_bridge
    bridge = CvBridge()

    # Setting up sensordata channel
    sensordata_channel = grpc.insecure_channel('localhost:50083')
    sensordata_stub = sensordata_pb2_grpc.SensordataStub(sensordata_channel)

    
    cv_image = cv2.imread('/home/thomas/dev/catkin_ws/src/ros_adapter/scripts/cat.jpg')
    cv_image = cv_image.reshape(549, 976, 3)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    msg = 0 
    try: 
        msg = bridge.cv2_to_imgmsg(cv_image, 'bgr8')
    except CvBridgeError as e:
        print(e)


    pub = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('streamer', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
