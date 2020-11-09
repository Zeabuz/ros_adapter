#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import grpc
import cv2

from sensordata import sensordata_pb2
from sensordata import sensordata_pb2_grpc

import numpy as np


class SomeSensordata():

    def stream_image():
        pass

    def stream_lidar():
        pass

    def stream_radar():
        pass


def streamer():

    # Setting up cv_bridge
    bridge = CvBridge()

    # Setting up sensordata channel
    sensordata_channel = grpc.insecure_channel('192.168.0.93:30052')
    sensordata_stub = sensordata_pb2_grpc.SensordataStub(sensordata_channel)

    pub = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('streamer', anonymous=True)
    #rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():

        for img_chunk in sensordata_stub.StreamSensordata(sensordata_pb2.SensordataRequest(operation="streaming")):
            buf = np.frombuffer(img_chunk.data, np.uint8)
            buf = buf.reshape(640, 800, 3)
            buf = cv2.flip(buf, 0)

        msg = 0
        try:
            msg = bridge.cv2_to_imgmsg(buf, 'rgb8')
        except CvBridgeError as e:
            print(e)

        pub.publish(msg)
        #rate.sleep()


if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
