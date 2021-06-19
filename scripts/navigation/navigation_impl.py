import rospy
import std_msgs.msg
import geometry_msgs.msg as geomsgs

import navigation.navigation_pb2
import navigation.navigation_pb2_grpc

from navigation.navigation_pb2 import NavigationResponse

class Navigation(navigation.navigation_pb2_grpc.NavigationServicer):
    def __init__(self, ego_pose_pub, target_pose_pubs,
                 ego_twist_pub, target_twist_pubs,
                 tf_pub, simulation_params, scenario_id):
        self.ego_pose_pub = ego_pose_pub
        self.target_pose_pubs = target_pose_pubs
        self.ego_twist_pub = ego_twist_pub
        self.target_twist_pubs = target_twist_pubs
        self.tf_pub = tf_pub
        self.simulation_params = simulation_params
        self.scenario_id = scenario_id

    def SendNavigationMessage(self, request, context):

        nav_header = std_msgs.msg.Header(
            frame_id=self.simulation_params["scenario" + self.scenario_id]["geo_frame"],
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
            self.ego_pose_pub.publish(pose_msg)
            transform.child_frame_id = "vessel_center"
        else:
            transform.child_frame_id = self.simulation_params["target_vessel_center_names"][vessel_name]
            self.target_pose_pubs[request.vesselName].publish(pose_msg)

        self.tf_pub.sendTransform(transform)

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

        self.ego_twist_pub.publish(twist_msg)

        return NavigationResponse(success=True)
