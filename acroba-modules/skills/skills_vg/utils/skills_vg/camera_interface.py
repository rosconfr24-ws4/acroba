#! /usr/bin/env python3

import rospy
import actionlib

import acroba_unity_msgs.msg
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np


class CameraControl(object):
    # create messages from feedback and result
    connected = False

    def __init__(self, global_namespace="", local_namespace="/unity_camera"):

        self._unityclient = actionlib.SimpleActionClient(
            global_namespace + local_namespace, acroba_unity_msgs.msg.RGBDSensorAction
        )
        self._subinfo = rospy.Subscriber(
            global_namespace + local_namespace + "/camera_info",
            CameraInfo,
            self.info_callback,
        )
        self.connected = self._unityclient.wait_for_server(rospy.Duration(10))
        self.intrinsic_matrix = None
        self.camera_width = None
        self.camera_height = None
        if not self.connected:
            rospy.logerr("Not possible to connect to the action server")
        else:
            rospy.loginfo("Connected to Unity Action Server")

    def capture(self):
        # check connection
        if not self.connected:
            # Try to connect again
            rospy.logwarn("Reconnecting to action server")
            self.connected = self.action_client.wait_for_server(rospy.Duration(secs=1))
            if not self.connected:
                rospy.logerr("Connection not possible")

        goal = acroba_unity_msgs.msg.RGBDSensorGoal()
        self._unityclient.send_goal(goal)
        self._unityclient.wait_for_result()
        return self._unityclient.get_result()

    def info_callback(self, camera_info):
        if camera_info is not None:
            self.intrinsic_matrix = np.eye(3)
            self.intrinsic_matrix[0, 0] = camera_info.K[0]
            self.intrinsic_matrix[0, 2] = camera_info.K[2]
            self.intrinsic_matrix[1, 1] = camera_info.K[4]
            self.intrinsic_matrix[1, 2] = camera_info.K[5]

            self.camera_height = camera_info.height
            self.camera_width = camera_info.width

    def depthmap_to_pointlist(self, depth_map):
        point_list = []
        for row in range(depth_map.shape[0]):
            for col in range(depth_map.shape[1]):
                x = (
                    depth_map[row, col]
                    * (col - self.intrinsic_matrix[0, 2])
                    / self.intrinsic_matrix[0, 0]
                )
                y = (
                    depth_map[row, col]
                    * (row - self.intrinsic_matrix[1, 2])
                    / self.intrinsic_matrix[1, 1]
                )
                point_list.append([x, y, depth_map[row, col]])
        return point_list


if __name__ == "__main__":
    rospy.init_node("camera_interface")
    camera_interface = CameraControl()
    rospy.loginfo("Is connected %i", camera_interface.connected)
    action_result = camera_interface.capture()
    print("Header of the image \n", action_result.image.header)
    print("Header of the depth image \n", action_result.depth_map.header)
    bridge_object = CvBridge()
    cv_rgb_image = bridge_object.imgmsg_to_cv2(
        action_result.image, desired_encoding="bgra8"
    )
    print("shape of rgb image", cv_rgb_image.shape)
    cv2.imshow("rgb_img", cv_rgb_image)
    cv2.waitKey(0)
    cv2.imwrite("rgb_img.png", cv_rgb_image)

    if action_result.depth_map.width > 0:
        cv_depth_image = bridge_object.imgmsg_to_cv2(
            action_result.depth_map, desired_encoding="32FC1"
        )
        print("depth shape", cv_depth_image.shape)
        cv2.imshow("depth_img", cv_depth_image)
        cv2.waitKey(0)
        cv2.imwrite("depth_img.png", cv_depth_image)
    else:
        print("Depth image not received")

    print("Intrinsic Matrix\n", camera_interface.intrinsic_matrix)

    pointcloud_list = camera_interface.depthmap_to_pointlist(cv_depth_image)
    print(len(pointcloud_list))
    import pcl

    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(pointcloud_list)
    print("Writting point cloud")
    pcl.save(pcl_cloud, "reconstructed_pointcloud.ply")

    """
    ## Old sample to receive pointcloud in the action server
    pc=point_cloud2.read_points(action_result.point_cloud,skip_nans=True,field_names=("x","y","z"))
    point_list=[]
    if pc is not None:
        for data in pc:
            point_list.append([data[0],data[1],data[2]])
        pcl_cloud=pcl.PointCloud()
        pcl_cloud.from_list(point_list)
        print("Number of points",len(point_list))
        if len(point_list)>0:
            pcl.save(pcl_cloud,"point_cloud.ply")
    else:
        print("PointCloud not received")
    """
