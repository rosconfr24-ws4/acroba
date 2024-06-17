#!/usr/bin/env python3
import numpy as np
from transforms3d import affines
from transforms3d import euler
from transforms3d import quaternions
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

#pip install transforms3d
#transfomrs3d works with rotation in radians

#in transforms3d, quaternions are[w,x,y,z]
#from geom
def geompose2mat(pose):
    T=[pose.position.x,pose.position.y,pose.position.z]
    q=[pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z]
    Q=quaternions.quat2mat(q)
    mat=affines.compose(T,Q,[1,1,1])
    return mat

def mat2geompose(mat):
    T,R,_,_=affines.decompose(mat)
    quad=quaternions.mat2quat(R)
    pose=Pose()
    pose.position.x=T[0]
    pose.position.y=T[1]
    pose.position.z=T[2]
    pose.orientation.x=quad[1]
    pose.orientation.y=quad[2]
    pose.orientation.z=quad[3]
    pose.orientation.w=quad[0]
    return pose

def euler2quat(euler_ang,axis='sxyz'):
    q=euler.euler2quat(euler_ang[0],euler_ang[1],euler_ang[2],axis)
    return q

def euler2quatRos(euler_ang,axis='sxyz'):
    q=euler.euler2quat(euler_ang[0],euler_ang[1],euler_ang[2],axis)
    q_ros=Quaternion()
    q_ros.w=q[0]
    q_ros.x=q[1]
    q_ros.y=q[2]
    q_ros.z=q[3]
    return q_ros

def quat2euler(quad):
    eul=euler.quat2euler(quad,'sxyz')
    return eul

def matFromPosEul(x,y,z,roll,pitch,yaw):
    T=[x,y,z]
    q=euler2quat([roll,pitch,yaw])
    Q=quaternions.quat2mat(q)
    mat=affines.compose(T,Q,[1,1,1])
    return mat

def inverseMatrix(mat_data):
    rot_inv=np.transpose(mat_data[:3,:3])
    x_inv=mat_data[0,3]*-1
    y_inv=mat_data[1,3]*-1
    z_inv=mat_data[2,3]*-1
    T_inv_false=np.array([x_inv,y_inv,z_inv])
    T_inv=np.matmul(rot_inv,T_inv_false)
    mat_inv=affines.compose(T_inv,rot_inv,[1,1,1])
    return mat_inv


if __name__ == '__main__':
    pose=Pose()
    pose.position.x=1
    pose.position.y=2
    pose.position.z=3
    quad=euler2quat([1,-1,1])
    print('initial quaternions',quad)
    pose.orientation.x=quad[1]
    pose.orientation.y=quad[2]
    pose.orientation.z=quad[3]
    pose.orientation.w=quad[0]

    mat=geompose2mat(pose)
    print('transformation matrix')
    print(mat)
    print('current positon',mat[0,3],mat[1,3],mat[2,3])
    geom2=mat2geompose(mat)
    print('position[',geom2.position,'] and orientation[',geom2.orientation,']')
    quad_vector=[geom2.orientation.w,geom2.orientation.x,geom2.orientation.y,geom2.orientation.z]
    print('orientation in euler',quat2euler(quad_vector))

    mat_data=matFromPosEul(1,2,3,1,-1,1)
    print(mat_data)

    mat_inv=inverseMatrix(mat_data)
    print("inverse matrix")
    print(mat_inv)

    identity=np.matmul(mat_data,mat_inv)
    print("identity")
    print(identity)

    offset=np.array([0,0,1,1])
    mat2=matFromPosEul(2,2,3,0,1.57,0)
    new_mat=np.matmul(mat2,offset)
    print(new_mat)

