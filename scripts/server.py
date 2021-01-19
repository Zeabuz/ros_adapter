#!/usr/bin/env python2

from concurrent import futures

import rospy

import std_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
#from ros_adapter.msg import RadarSpoke

from cv_bridge import CvBridge, CvBridgeError

import grpc
import cv2

from sensor_streaming import sensor_streaming_pb2
from sensor_streaming import sensor_streaming_pb2_grpc

import numpy as np

class SensorStreaming(sensor_streaming_pb2_grpc.SensorStreamingServicer):
    def __init__(self, camera_pubs, lidar_pub, radar_pub):
        print("creating")
        self.bridge = CvBridge()
        self.camera_pubs = camera_pubs
        self.lidar_pub = lidar_pub
        self.radar_pub = radar_pub

    def StreamCameraSensor(self, request, context):
        """
        Takes in a gRPC SensorStreamingRequest containing
        all the data needed to create and publish a sensor_msgs/Image
        ROS message.
        """
        img_string = request.data

        cv_image = np.fromstring(img_string, np.uint8)

        # Backward for some wierd reason
        cv_image = cv_image.reshape(request.height, request.width, 3)
        cv_image = cv2.flip(cv_image, 0)

        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        msg = Image()
        header = std_msgs.msg.Header()
        try:
            # RGB
            #msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

            # BGR
            msg = self.bridge.cv2_to_imgmsg(bgr_image, 'bgr8')

            header.stamp = rospy.Time.from_sec(request.timeStamp)
            msg.header = header
        except CvBridgeError as e:
            print(e)

        camera_pubs[request.frame_id].publish(msg)

        return sensor_streaming_pb2.CameraStreamingResponse(success=True)

    def StreamLidarSensor(self, request, context):
        """
        Takes in a gRPC LidarStreamingRequest containing
        all the data needed to create and publish a PointCloud2
        ROS message.
        """

        pointcloud_msg = PointCloud2()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.from_sec(request.timeInSeconds)
        
        header.frame_id = "velodyne"
        pointcloud_msg.header = header

        pointcloud_msg.height = request.height
        pointcloud_msg.width = request.width

        fields = request.fields

        # Set PointCloud[] fields in pointcloud_msg
        for i in range(len(fields)):
            pointcloud_msg.fields.append(PointField())
            pointcloud_msg.fields[i].name = fields[i].name
            pointcloud_msg.fields[i].offset = fields[i].offset
            pointcloud_msg.fields[i].datatype = fields[i].datatype
            pointcloud_msg.fields[i].count = fields[i].count

        pointcloud_msg.is_bigendian = request.isBigEndian
        pointcloud_msg.point_step = request.point_step
        pointcloud_msg.row_step = request.row_step

        pointcloud_msg.data = request.data

        pointcloud_msg.is_dense = request.is_dense

        self.lidar_pub.publish(pointcloud_msg)

        return sensor_streaming_pb2.LidarStreamingResponse(success=True)


    def StreamRadarSensor(self, request, context):
        """
        Takes in a gRPC RadarStreamingRequest containing
        all the data needed to create and publish a RadarSpoke
        ROS message.
        """

        #number_of_spokes = request.numSpokes

        #for i in range(number_of_spokes):

        #    radar_spoke_msg = RadarSpoke()

        #    # Header
        #    header = std_msgs.msg.Header()
        #    header.frame_id = "milliampere_radar"
        #    header.stamp = rospy.Time.from_sec(request.timeInSeconds[i])
        #    radar_spoke_msg.azimuth = request.azimuth[i]
        #    radar_spoke_msg.intensity = request.radarSpokes[i * request.numSamples : i * request.numSamples + request.numSamples]

        #    radar_spoke_msg.range_start = request.rangeStart
        #    radar_spoke_msg.range_increment = request.rangeIncrement
        #    radar_spoke_msg.min_intensity = request.minIntensity
        #    radar_spoke_msg.max_intensity = request.maxIntensity
        #    radar_spoke_msg.num_samples = request.numSamples

        #    self.radar_pub.publish(radar_spoke_msg)

        return sensor_streaming_pb2.RadarStreamingResponse(success=True)


def serve(camera_pubs, lidar_pub, radar_pub):
    # Linux Desktop
    ip = '127.0.0.1'

    # Desktop VM
    #ip = '192.168.0.116'

    # Docker Container
    #ip = '172.18.0.22'

    port = '30052'
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(SensorStreaming(camera_pubs, lidar_pub, radar_pub), server)
    server.add_insecure_port(ip + ':' + port)
    print(ip + ":" + port)
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':

    cam_ids = ["F", "FL", "FR", "RL", "RR"]
    camera_pubs = dict()
    for cam_id in cam_ids:
        camera_pubs[cam_id] = rospy.Publisher('EO/' + cam_id + '/image_raw',
                                              Image, queue_size=10)

    lidar_pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)

    # TODO: Change the message type to be published
    radar_pub = rospy.Publisher('radar/driver/spokes', 
                                Image, 
                                queue_size=10)

    rospy.init_node('syntetic_data', anonymous=True)
    serve(camera_pubs, lidar_pub, radar_pub)
