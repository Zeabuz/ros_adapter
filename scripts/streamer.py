#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import grpc

import sensordata_pb2
import sensordata_pb2_grpc

def streamer():

    # Setting up sensordata channel
    sensordata_channel = grpc.insecure_channel('localhost:50083')
    sensordata_stub = sensordata_pb2_grpc.SensordataStub(sensordata_channel)

    pub = rospy.Publisher('image', String, queue_size=10)
    rospy.init_node('streamer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world " + str(rospy.get_time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
