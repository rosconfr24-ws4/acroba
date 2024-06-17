#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import skills.msg
import math
import time
import random
import copy
from tf.transformations import quaternion_from_euler, quaternion_multiply
import geometry_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from acroba_unity_msgs.msg import GripperControlCommand


class Global:
    rgb_image = None
    depth_image = None
    camera_info = None


def rgb_callback(data):
    Global.rgb_image = data


def depth_callback(data):
    Global.depth_image = data


def camera_info_callback(data):
    Global.camera_info = data


def main():
    t2 = time.time()

    rospy.init_node("test_skills", anonymous=True)
    rgb_topic = "/camera/color/image_raw"
    depth_topic = "/camera/aligned_depth_to_color/image_raw"
    camera_info_topic = "/camera/color/camera_info"

    rospy.Subscriber(rgb_topic, Image, rgb_callback)
    rospy.Subscriber(depth_topic, Image, depth_callback)
    rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_callback)

    client_move_to = actionlib.SimpleActionClient("MoveTo", skills.msg.MoveToAction)
    client_sam_matching_gen_grasp = actionlib.SimpleActionClient(
        "SamMatchGenGrasp", skills.msg.SamMatchGenGraspAction
    )
    client_crop_image = actionlib.SimpleActionClient(
        "CropImage", skills.msg.CropImageAction
    )
    client_reset_gripper = actionlib.SimpleActionClient(
        "ResetGripper", skills.msg.ResetGripperAction
    )
    client_grasp = actionlib.SimpleActionClient("Grasp", skills.msg.GraspAction)
    client_release = actionlib.SimpleActionClient("Release", skills.msg.ReleaseAction)

    pi = math.pi

    n_max = 3

    print("waiting for servers")
    client_move_to.wait_for_server()
    client_sam_matching_gen_grasp.wait_for_server()
    client_crop_image.wait_for_server()
    client_grasp.wait_for_server()
    client_release.wait_for_server()
    client_reset_gripper.wait_for_server()
    print("servers ready")

    # w8 for data
    rate = rospy.Rate(60)

    while (
        Global.rgb_image is None
        or Global.depth_image is None
        or Global.camera_info is None
    ) and not rospy.is_shutdown():
        rospy.loginfo("Waiting for camera topics to be publish")
        rate.sleep()

    # STEP0: GO TO HOME POSITION AND RESET GRIPPER
    # Go to home position
    goal_move_to_home_position = skills.msg.MoveToGoal()
    starting_pose = [
        22 * pi / 180,
        -111 * pi / 180,
        -88 * pi / 180,
        -70 * pi / 180,
        85 * pi / 180,
        -73 * pi / 180,
    ]

    goal_move_to_home_position.velocity_factor = 1
    goal_move_to_home_position.acceleration_factor = 1
    goal_move_to_home_position.articular_pose = starting_pose
    goal_move_to_home_position.cartesian = False
    goal_move_to_home_position.frame_id = "base"  # UR5 base frame
    client_move_to.send_goal(goal_move_to_home_position)
    client_move_to.wait_for_result()
    print("robot at starting pose")

    # Open gripper
    goal_reset_gripper = skills.msg.ResetGripperGoal()
    client_reset_gripper.send_goal(goal_reset_gripper)
    client_reset_gripper.wait_for_result()

    for i in range(0, n_max):
        if rospy.is_shutdown():
            break

        # STEP1 : crop images
        goal_crop_image = skills.msg.CropImageGoal()

        goal_crop_image.camera_info = Global.camera_info

        goal_crop_image.left_crop = 0
        goal_crop_image.right_crop = 640
        goal_crop_image.top_crop = 0
        goal_crop_image.bottom_crop = 400

        goal_crop_image.image = Global.rgb_image
        client_crop_image.send_goal(goal_crop_image)
        client_crop_image.wait_for_result()
        result = client_crop_image.get_result()
        rgb_image_cropped = result.cropped_image

        goal_crop_image.image = Global.depth_image
        client_crop_image.send_goal(goal_crop_image)
        client_crop_image.wait_for_result()
        result = client_crop_image.get_result()
        depth_image_cropped = result.cropped_image

        camera_info_cropped_image = result.cropped_image_camera_info

        # STEP2: LOCATE the Part to be grasped in the scene

        t = time.time()

        goal_sam_matching_gen_grasp = skills.msg.SamMatchGenGraspGoal()
        goal_sam_matching_gen_grasp.templates_path = "/home/acroba/share/temp/src/skills/28-Sep-2023_(15:27:53.047609)/templates.pt"  # cube
        goal_sam_matching_gen_grasp.result_path = "/home/acroba/acroba/share/temp"
        goal_sam_matching_gen_grasp.num_max_dets = 1
        goal_sam_matching_gen_grasp.tresh = 0.2
        goal_sam_matching_gen_grasp.visualize = True
        goal_sam_matching_gen_grasp.save_json = True
        goal_sam_matching_gen_grasp.rgb_image = rgb_image_cropped
        goal_sam_matching_gen_grasp.depth_image = depth_image_cropped
        goal_sam_matching_gen_grasp.camera_info = camera_info_cropped_image

        client_sam_matching_gen_grasp.send_goal(goal_sam_matching_gen_grasp)
        client_sam_matching_gen_grasp.wait_for_result()
        result_sam_matching_gen_grasp = client_sam_matching_gen_grasp.get_result()
        print("object located")
        rospy.loginfo("Succeeded in %s seconds" % (time.time() - t))

        # STEP3: GO TO PICK APPROACH THEN PICK
        pickZApproach = 0.10
        pickApproach = geometry_msgs.msg.Pose()
        pickApproach.position.x = result_sam_matching_gen_grasp.grasping_pose_list[
            0
        ].position.x
        pickApproach.position.y = result_sam_matching_gen_grasp.grasping_pose_list[
            0
        ].position.y
        pickApproach.position.z = (
            result_sam_matching_gen_grasp.grasping_pose_list[0].position.z
            - pickZApproach
        )
        pickApproach.orientation.x = result_sam_matching_gen_grasp.grasping_pose_list[
            0
        ].orientation.x
        pickApproach.orientation.y = result_sam_matching_gen_grasp.grasping_pose_list[
            0
        ].orientation.y
        pickApproach.orientation.z = result_sam_matching_gen_grasp.grasping_pose_list[
            0
        ].orientation.z
        pickApproach.orientation.w = result_sam_matching_gen_grasp.grasping_pose_list[
            0
        ].orientation.w

        pick = copy.deepcopy(pickApproach)
        pick.position.z += pickZApproach - 0.01

        goal_move_to_pick = skills.msg.MoveToGoal()
        goal_move_to_pick.points.append(pickApproach)
        goal_move_to_pick.points.append(pick)
        goal_move_to_pick.velocity_factor = 1
        goal_move_to_pick.acceleration_factor = 1
        goal_move_to_pick.frame_id = Global.rgb_image.header.frame_id
        goal_move_to_pick.endeffector = "endeffector"
        goal_move_to_pick.cartesian = True
        client_move_to.send_goal(goal_move_to_pick)
        client_move_to.wait_for_result()
        print("robot at pick approach pose")

        # Close gripper
        goal_grasp = skills.msg.GraspGoal()
        client_grasp.send_goal(goal_grasp)
        client_grasp.wait_for_result()

        # STEP4: GO TO HOME POSE
        client_move_to.send_goal(goal_move_to_home_position)
        client_move_to.wait_for_result()
        print("robot at starting pose")

        # STEP5: GO TO PLACE POSE
        placing_pose = [
            -19 * pi / 180,
            -115 * pi / 180,
            -90 * pi / 180,
            -50 * pi / 180,
            95 * pi / 180,
            -116 * pi / 180,
        ]

        goal_move_to_place = skills.msg.MoveToGoal()
        goal_move_to_place.articular_pose = placing_pose
        goal_move_to_place.cartesian = False
        goal_move_to_place.velocity_factor = 1
        goal_move_to_place.acceleration_factor = 1
        goal_move_to_place.frame_id = "base"  # UR5 base frame
        client_move_to.send_goal(goal_move_to_place)
        client_move_to.wait_for_result()
        print("robot at place pose")

        # Open gripper
        goal_release = skills.msg.ReleaseGoal()
        client_release.send_goal(goal_release)
        client_release.wait_for_result()

        # STEP6: GO TO HOME POSE
        goal_move_to_home_position.velocity_factor = 0.5
        goal_move_to_home_position.acceleration_factor = 0.5
        client_move_to.send_goal(goal_move_to_home_position)
        client_move_to.wait_for_result()
        print("robot at starting pose")
        rospy.sleep(2)

    print("finish")

    rospy.loginfo("Succeeded in %s seconds" % (time.time() - t2))


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
