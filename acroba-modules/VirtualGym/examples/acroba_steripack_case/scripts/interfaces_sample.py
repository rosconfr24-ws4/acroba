#!/usr/bin/env python3
import rospy
from acroba_gym.camera_interface import CameraControl
from acroba_steripack_case.printer_interface import PrinterInterface
from acroba_steripack_case.motor_interface import MotorInterface

import cv2
from cv_bridge import CvBridge

if __name__=="__main__":
    
    rospy.init_node("steripack_sample")
    rospy.sleep(1)
    #Initialize interfaces
    zivid_camera=CameraControl(local_namespace="/zivid_camera")
    intel_camera=CameraControl(local_namespace="/intel_camera")
    
    motor_interface=MotorInterface()
    printer_interface=PrinterInterface()
    rospy.sleep(3)
    
    
    motor_interface.move_right()
    rospy.sleep(3)
    print("===ZIVID CAMERA====")
    action_result=zivid_camera.capture()
    bridge_object=CvBridge()
    cv_rgb_image=bridge_object.imgmsg_to_cv2(action_result.image,desired_encoding="bgra8")
    print("shape of rgb image",cv_rgb_image.shape)
    cv2.imshow("rgb_img",cv_rgb_image)
    cv2.waitKey(500)
    
    print("=== INTEL CAMERA====")
    action_result=intel_camera.capture()
    bridge_object=CvBridge()
    cv_rgb_image=bridge_object.imgmsg_to_cv2(action_result.image,desired_encoding="bgra8")
    print("shape of rgb image",cv_rgb_image.shape)
    cv2.imshow("rgb_img",cv_rgb_image)
    cv2.waitKey(500)
    
    #print
    rospy.loginfo("print")
    printer_interface.print_action()
    printer_interface.waitTimeout(10)