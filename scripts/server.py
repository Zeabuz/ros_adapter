#!/usr/bin/env python2

import sys
import yaml

from concurrent import futures

import rospy
import roslib
import rosparam
from rosgraph_msgs.msg import Clock

import geometry_msgs.msg as geomsgs

import tf2_ros
import tf.transformations as tftrans

import std_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from ros_adapter.msg import RadarSpoke

from cv_bridge import CvBridge, CvBridgeError

import grpc
import cv2

from config import parser

from sensor_streaming import sensor_streaming_pb2
from sensor_streaming import sensor_streaming_pb2_grpc
from navigation import navigation_pb2
from navigation import navigation_pb2_grpc

import numpy as np
import pdb

class SensorStreaming(sensor_streaming_pb2_grpc.SensorStreamingServicer):
    def __init__(self, camera_pubs, lidar_pub, radar_pub, clock_pub):
        print("creating")
        self.bridge = CvBridge()
        self.camera_pubs = camera_pubs
        self.lidar_pub = lidar_pub
        self.radar_pub = radar_pub
        self.clock_pub = clock_pub

    def StreamCameraSensor(self, request, context):
        """
        Takes in a gRPC SensorStreamingRequest containing
        all the data needed to create and publish a sensor_msgs/Image
        ROS message.
        """
        img_string = request.data

        cv_image = np.fromstring(img_string, np.uint8)

        # NOTE, the height is specifiec as a parameter before the width
        cv_image = cv_image.reshape(request.height, request.width, 3)
        cv_image = cv2.flip(cv_image, 0)

        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        msg = Image()
        header = std_msgs.msg.Header()
        try:
            # RGB
            # msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

            # BGR
            msg = self.bridge.cv2_to_imgmsg(bgr_image, 'bgr8')

            header.stamp = rospy.Time.from_sec(request.timeStamp)
            msg.header = header
        except CvBridgeError as e:
            print(e)

        camera_pubs[request.frame_id.encode("ascii", 'ignore')].publish(msg)

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

        # TODO: This does not belong in this RPC implementation, should be
        # moved to own or something like that.
        sim_clock = Clock()
        sim_clock.clock = rospy.Time.from_sec(request.timeInSeconds)
        self.clock_pub.publish(sim_clock)

        return sensor_streaming_pb2.LidarStreamingResponse(success=True)

    def StreamRadarSensor(self, request, context):
        """
        Takes in a gRPC RadarStreamingRequest containing
        all the data needed to create and publish a RadarSpoke
        ROS message.
        """

        number_of_spokes = request.numSpokes

        for i in range(number_of_spokes):

            radar_spoke_msg = RadarSpoke()

            # Header
            header = std_msgs.msg.Header()
            header.frame_id = "milliampere_radar"
            header.stamp = rospy.Time.from_sec(request.timeInSeconds[i])
            radar_spoke_msg.azimuth = request.azimuth[i]
            radar_spoke_msg.intensity = request.radarSpokes[
                    i * request.numSamples:
                    i * request.numSamples + request.numSamples]

            radar_spoke_msg.range_start = request.rangeStart
            radar_spoke_msg.range_increment = request.rangeIncrement
            radar_spoke_msg.min_intensity = request.minIntensity
            radar_spoke_msg.max_intensity = request.maxIntensity
            radar_spoke_msg.num_samples = request.numSamples

            self.radar_pub.publish(radar_spoke_msg)

        return sensor_streaming_pb2.RadarStreamingResponse(success=True)


class Navigation(navigation_pb2_grpc.NavigationServicer):
    def __init__(self, ego_pose_pub, target_pose_pubs,
                 ego_twist_pub, target_twist_pubs, tf_pub):
        self.ego_pose_pub = ego_pose_pub
        self.target_pose_pubs = target_pose_pubs
        self.ego_twist_pub = ego_twist_pub
        self.target_twist_pubs = target_twist_pubs
        self.tf_pub = tf_pub

    def SendNavigationMessage(self, request, context):

        # TODO: This frame_id should be dynamically set from a config file.
        nav_header = std_msgs.msg.Header(
            frame_id=simulation_params["scenario" + scenario_id]["geo_frame"],
            stamp=rospy.Time.from_sec(request.timeStamp)
        )

        transform = geomsgs.TransformStamped()
        transform.header = nav_header

        position = geomsgs.Point()
        position.x = request.position.x
        position.y = request.position.y
        position.z = request.position.z

        orientation = geomsgs.Quaternion()
        orientation.x = request.orientation.x
        orientation.y = request.orientation.y
        orientation.z = request.orientation.z
        orientation.w = request.orientation.w

        pose_msg = geomsgs.PoseStamped(
            header=nav_header,
            pose=geomsgs.Pose(
                position=position,
                orientation=orientation
            )
        )

        transform.transform = geomsgs.Transform(
                translation=position,
                rotation=orientation
                )

        vessel_name = request.vesselName

        if vessel_name == "ego":
            ego_pose_pub.publish(pose_msg)
            transform.child_frame_id = "vessel_center"
        else:
            transform.child_frame_id = simulation_params["target_vessel_center_names"][vessel_name]
            target_pose_pubs[request.vesselName].publish(pose_msg)

        tf_pub.sendTransform(transform)

        linear_vel = geomsgs.Vector3()
        linear_vel.x = request.linearVelocity.x
        linear_vel.y = request.linearVelocity.y
        linear_vel.z = request.linearVelocity.z

        angular_vel = geomsgs.Vector3()
        angular_vel.x = request.angularVelocity.x
        angular_vel.y = request.angularVelocity.y
        angular_vel.z = request.angularVelocity.z

        twist_msg = geomsgs.TwistStamped(
            header=nav_header,
            twist=geomsgs.Twist(
                linear=linear_vel,
                angular=angular_vel
            )
        )

        ego_twist_pub.publish(twist_msg)

        return navigation_pb2.NavigationResponse(success=True)


def serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          ego_pose_pub, target_pose_pubs,
          ego_twist_pub, target_twist_pubs, tf_pub):

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
            SensorStreaming(camera_pubs, lidar_pub, radar_pub, clock_pub),
            server)

    navigation_pb2_grpc.add_NavigationServicer_to_server(
        Navigation(ego_pose_pub, target_pose_pubs,
                   ego_twist_pub, target_twist_pubs, tf_pub),
        server)

    server.add_insecure_port(server_ip + ':' + str(server_port))
    print(server_ip + ":" + str(server_port))
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':

    rospy.init_node('syntetic_data', anonymous=True)

    config_dir_path = sys.argv[1]
    scenario_id = sys.argv[2]

    simulation_params = parser.parse_scenario_config(config_dir_path, scenario_id)

    server_ip = simulation_params["server_ip"]
    server_port = simulation_params["server_port"]

    ego_vessel_name = simulation_params["scenario" + scenario_id]["ego_vessel_name"]
    target_vessel_names = simulation_params["target_vessel_names"]

    cam_ids = simulation_params['ego_optical_camera_frame_ids']
    camera_pubs = dict()
    for cam_id in cam_ids:
        camera_pubs[cam_id] = rospy.Publisher(
                simulation_params['ego_optical_camera_prefix'] +
                cam_id +
                simulation_params['ego_optical_camera_postfix'],
                Image, queue_size=10)

    lidar_pub = rospy.Publisher(simulation_params['ego_lidar_topic'],
                                PointCloud2,
                                queue_size=10)

    radar_pub = rospy.Publisher(simulation_params['ego_radar_topic'],
                                RadarSpoke,
                                queue_size=10)

    clock_pub = rospy.Publisher('clock', Clock, queue_size=10)

    ego_pose_pub = rospy.Publisher(
            simulation_params['ego_pose_topic'],
            geomsgs.PoseStamped, queue_size=10)

    ego_twist_pub = rospy.Publisher(
            simulation_params['ego_twist_topic'],
            geomsgs.TwistStamped, queue_size=10)

    target_pose_pubs = dict()
    for target_name in target_vessel_names:
        target_pose_pubs[target_name] = rospy.Publisher(
                simulation_params['target_vessel_pose_topics'][target_name],
                geomsgs.PoseStamped,
                queue_size=10)

    target_twist_pubs = dict()
    for target_name in target_vessel_names:
        target_twist_pubs[target_name] = rospy.Publisher(
                simulation_params['target_vessel_twist_topics'][target_name],
                geomsgs.TwistStamped,
                queue_size=10)

    tf_pub = tf2_ros.TransformBroadcaster()

    serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          ego_pose_pub, target_pose_pubs,
          ego_twist_pub, target_twist_pubs, tf_pub)
