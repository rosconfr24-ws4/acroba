#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import String
from acroba_unity_msgs.msg import ModelStates
from tf.transformations import *
import math
from geometry_msgs.msg import Pose,Point
import copy


class UnitySceneManager(object):
    """
    Class that communicate with the scene and interact with it: get the model pose, set model pose
    
    """
    def __init__(self, unity_model_list=None):
        """[summary]

        Args:
            unity_model_list ([string list], optional): [description]. Models to search for, if None use all models.
        """
        self._model_pose_dict = {}
        self._unity_model_list = unity_model_list
        self.model_state_topic="/model_states"
        self._set_models_states=ModelStates()
        # We now start the suscriber once we have the indexes of each model
        self._sub=rospy.Subscriber(self.model_state_topic, ModelStates, self.callback,queue_size=1)
        self._model_pub=rospy.Publisher("/set_model_states", ModelStates, queue_size=2)
        self._add_model_pub=rospy.Publisher("/add_model",ModelStates,queue_size=1)
        self._add_frame_pub=rospy.Publisher("/add_frame",ModelStates,queue_size=1)
        self._remove_model_pub=rospy.Publisher("/remove_model",ModelStates,queue_size=1)
        self.is_connected=False
        self.raw_model_data=None
        self.check_connections()
    
    def check_connections(self):
        while not bool(self._model_pose_dict) and not rospy.is_shutdown() and not self.is_connected:
            try:
                state_msg = rospy.wait_for_message(self.model_state_topic, ModelStates, timeout=5.0)
                rospy.logdebug("Current "+self.model_state_topic+" READY=>")
                self.is_connected=True
            except Exception as e: 
                print(e)
                rospy.logerr("Current "+self.model_state_topic+" not ready yet, retrying...")
    
        
    def find_element_in_list(self,element, list_element):
        try:
            index_element = list_element.index(element)
            return index_element
        except ValueError:
            return -1
    
    def set_models_states(self,name_list,pose_list):
        state_msg=ModelStates()
        state_msg.name=name_list
        state_msg.pose=pose_list
        self._model_pub.publish(state_msg)
        
    def remove_models(self,name_list):
        state_msg=ModelStates()
        state_msg.name=name_list
        self._remove_model_pub.publish(state_msg)

    #Save the data
    def callback(self, data):
        self.raw_model_data=data
        self._model_pose_dict.clear()
        self.is_connected=True
        if self._unity_model_list is not None:
            for model_name in self._unity_model_list:
                # Retrieve the corresponding index
                model_name_found = self.find_element_in_list(model_name,data.name)
                if model_name_found>=0:
                    self._model_pose_dict[model_name]=data.pose[model_name_found]
        else:
            for i  in range(len(data.name)):
                self._model_pose_dict[data.name[i]]=data.pose[i]
                
    def get_all_models_dict(self):
        if bool(self._model_pose_dict):
            return copy.deepcopy(self._model_pose_dict)
        else:
            None
            
    def get_model_pose(self, model_name):

        pose_now = None

        try:
            pose_now = self._model_pose_dict[model_name]
        except Exception as e:
            s = str(e)
            rospy.loginfo("Error, The _robots_models_dict is not ready = " + s)

        return pose_now

    #Add models relative to the prefab folder of the unity project
    def add_models(self,names_list,pose_list,scale_list=None):
        """Add mesh models that are relative to the prefab folder

        Args:
            names_list ([string]): list of relative name from the prefab folder
            pose_list ([type]): list of poses
        """
        state_msg=ModelStates()
        state_msg.name=names_list
        state_msg.pose=pose_list
        if scale_list is not None:
            state_msg.scale=scale_list
        self._add_model_pub.publish(state_msg)

    #Add models relative to the prefab folder of the unity project
    def add_frame(self,name,pose,ref_frame_list=""):
        """Add mesh models that are relative to the prefab folder

        Args:
            names_list ([string]): list of relative name from the prefab folder
            pose_list ([type]): list of poses
        """
        state_msg=ModelStates()
        state_msg.name=[name]
        state_msg.pose=[pose]
        state_msg.header.frame_id = ref_frame_list
        self._add_frame_pub.publish(state_msg)
        rospy.sleep(0.5)

def test():
    rospy.init_node('unity_scene_manager', anonymous=True, log_level=rospy.DEBUG)
    
    #unity_model_list=['Target']
    scene_manager=UnitySceneManager()
    '''
    rate = rospy.Rate(1)  # 10hz
    count=0
    while not rospy.is_shutdown() and count<10:
        count=count+1
        for model_name in unity_model_list:
            pose_now = scene_manager.get_model_pose(model_name)
            if pose_now is not None:
                print(type(pose_now))
                print ("POSE NOW ROBOT =" + model_name + "==>" + str(pose_now))
                pose_now.position.x=pose_now.position.x+0.05
                scene_manager.set_models_states([model_name],[pose_now])
        rate.sleep()
'''
    rospy.sleep(1)
    model_dict=scene_manager.get_all_models_dict()
    print("model dict")
    print(model_dict)
    names=["pth2_box","pth2_box"]
    pose_msg1=Pose()
    pose_msg1.position.x=1
    pose_msg1.position.y=1
    pose_msg1.position.z=0.01
    q_1=quaternion_from_euler(0,0,0)
    pose_msg1.orientation.x=q_1[0]
    pose_msg1.orientation.y=q_1[1]
    pose_msg1.orientation.z=q_1[2]
    pose_msg1.orientation.w=q_1[3]
    pose_msg2=Pose()
    pose_msg2.position.x=0.9
    pose_msg2.position.y=1
    pose_msg2.position.z=0.01
    q_2=quaternion_from_euler(0,-math.pi/2,0)
    pose_msg2.orientation.x=q_2[0]
    pose_msg2.orientation.y=q_2[1]
    pose_msg2.orientation.z=q_2[2]
    pose_msg2.orientation.w=q_2[3]
    print("Add model pth2_models without scale")
    scene_manager.add_models(names,[pose_msg1,pose_msg2])
    
    ## Add obstacles with scaling
    scale1=Point()
    scale1.x=5
    scale1.y=5
    scale1.z=5
    pose_msg1.position.y=-1
    pose_msg1.position.x=1.1
    scale2=Point()
    scale2.z=2
    pose_msg2.position.y=-0.8
    
    pose_msg3=copy.deepcopy(pose_msg1)
    pose_msg3.position.y=-1.2
    scale3=Point()
    scale3.x=1;scale3.y=1;scale3.z=2;
    
    pose_msg4=copy.deepcopy(pose_msg1)
    pose_msg4.position.y=-1.3
    scale4=Point()
    scale4.x=1;scale4.y=2;scale4.z=1;
    
    pose_msg5=copy.deepcopy(pose_msg1)
    pose_msg5.position.y=-1.4
    scale5=Point()
    scale5.x=2;scale5.y=1;scale5.z=1;
    
    names=["pth2_box","pth2_box","pth2_box","pth2_box","pth2_box"]
    rospy.sleep(2)
    print("Add model pth2_models with scale")
    scene_manager.add_models(names,[pose_msg1,pose_msg2,pose_msg3,pose_msg4,pose_msg5],[scale1,scale2,scale3,scale4,scale5])
    
    ## Add models with orientation
    pose_msg1=Pose()
    pose_msg1.position.x=0
    pose_msg1.position.y=1.1
    pose_msg1.position.z=0.01
    q_1=quaternion_from_euler(0,0,0)
    pose_msg1.orientation.x=q_1[0];pose_msg1.orientation.y=q_1[1];
    pose_msg1.orientation.z=q_1[2];pose_msg1.orientation.w=q_1[3];
    print("============ Orientacioens  =============")
    print("Orientation 1",pose_msg1.orientation)
    pose_msg2=copy.deepcopy(pose_msg1)
    q_2=quaternion_from_euler(0,0,math.pi/2)
    pose_msg2.orientation.x=q_2[0];pose_msg2.orientation.y=q_2[1];
    pose_msg2.orientation.z=q_2[2];pose_msg2.orientation.w=q_2[3];
    print("Orientation 2",pose_msg2.orientation)
    pose_msg2.position.y=1.2
    
    pose_msg3=copy.deepcopy(pose_msg1)
    q_3=quaternion_from_euler(0,math.pi/2,0)
    pose_msg3.orientation.x=q_3[0];pose_msg3.orientation.y=q_3[1];
    pose_msg3.orientation.z=q_3[2];pose_msg3.orientation.w=q_3[3];
    print("Orientation 3",pose_msg3.orientation)
    pose_msg3.position.y=1.3
    
    pose_msg4=copy.deepcopy(pose_msg1)
    q_4=quaternion_from_euler(math.pi/2,0,0)
    pose_msg4.orientation.x=q_4[0];pose_msg4.orientation.y=q_4[1];
    pose_msg4.orientation.z=q_4[2];pose_msg4.orientation.w=q_4[3];
    print("Orientation 4",pose_msg4.orientation)
    pose_msg4.position.y=1.4
    
    
    scale1.x=2;scale1.y=2;scale1.z=2
    scales=[scale1,scale1,scale1,scale1]
    poses=[pose_msg1,pose_msg2,pose_msg3,pose_msg4]
    names=["pth2_box","pth2_box","pth2_box","pth2_box"]
    rospy.sleep(1)
    print("Add model models with orientation")
    scene_manager.add_models(names,poses,scales)
    rospy.sleep(1)
    ## Read pth orientations
    model_dict=scene_manager.get_all_models_dict()
    for key,value in model_dict.items():
        if "pth2" in key:
            print("Model ",key,"with orientation: ",value.orientation)
    
def test2():
    rospy.init_node('listener', anonymous=True, log_level=rospy.DEBUG)
    
    #unity_model_list=['Target']
    scene_manager=UnitySceneManager()
    rospy.sleep(1)
    name="l_gripping_point"
    pose_msg1=Pose()
    pose_msg1.position.x=0
    pose_msg1.position.y=0
    pose_msg1.position.z=0
    q_1=quaternion_from_euler(0,0,0)
    pose_msg1.orientation.x=q_1[0]
    pose_msg1.orientation.y=q_1[1]
    pose_msg1.orientation.z=q_1[2]
    pose_msg1.orientation.w=q_1[3]
    print("Add frame")
    scene_manager.add_frame("l_gripping_point",pose_msg1,'l_finger_link')
    
    scene_manager.add_frame("r_gripping_point",pose_msg1,'r_finger_link') 

    model_dict=scene_manager.get_all_models_dict()
    print("model dict")
    print(model_dict)

if __name__ == '__main__':
    test()