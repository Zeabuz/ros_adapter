#!/usr/bin/env python3

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
from sensor_streaming import sensor_streaming_impl

from navigation import navigation_pb2
from navigation import navigation_pb2_grpc
from navigation import navigation_impl

import numpy as np


def serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          ego_pose_pub, target_pose_pubs,
          ego_twist_pub, target_twist_pubs,
          tf_pub, simulation_params, scenario_id):

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
            sensor_streaming_impl.SensorStreaming(camera_pubs, lidar_pub, radar_pub, clock_pub),
            server)

    navigation_pb2_grpc.add_NavigationServicer_to_server(
        navigation_impl.Navigation(
            ego_pose_pub, target_pose_pubs,
            ego_twist_pub, target_twist_pubs,
            tf_pub, simulation_params, scenario_id),
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
          ego_twist_pub, target_twist_pubs,
          tf_pub, simulation_params, scenario_id)
