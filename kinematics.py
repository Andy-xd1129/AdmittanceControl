"""
UR5正运动学计算函数（使用改进DH参数）
"""
import numpy as np
from math import cos, sin, pi
from math import atan2, sqrt, degrees
from config import UR5_DH_PARAMS

def dh_matrix(alpha, a, d, theta):
    """计算改进DH变换矩阵"""
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)
    
    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa, ca, d*ca],
        [0, 0, 0, 1]
    ])

def ur5_forward_kinematics(joint_angles):
    """计算UR5的正运动学（使用改进DH参数）"""
    T = np.eye(4)
    
    for i in range(6):
        alpha, a, d, theta_offset = UR5_DH_PARAMS[i]
        theta = joint_angles[i] + theta_offset
        
        Ti = dh_matrix(alpha, a, d, theta)
        T = np.dot(T, Ti)
        
    return T

def get_ur5_position(joint_angles):
    """获取末端执行器位置"""
    T = ur5_forward_kinematics(joint_angles)
    return T[:3, 3]

import numpy as np
from math import atan2, sqrt, pi, degrees, sin

def get_ur5_orientation(joint_angles):
    """
    获取末端执行器的旋转矩阵（唯一确定姿态）
    :param joint_angles: 关节角度列表(弧度)
    :return: 3x3 旋转矩阵 numpy.ndarray
    """
    T = ur5_forward_kinematics(joint_angles)  # 获取齐次变换矩阵
    return T[:3, :3]  # 提取旋转矩阵部分

def euler_to_matrix(euler_angles):
    """
    将ZYX欧拉角(来自CoppeliaSim)转换为旋转矩阵（固定坐标系外旋）
    :param euler_angles: [gamma, beta, alpha] 弧度 (绕Z, Y, X轴)
    :return: 3x3 旋转矩阵
    """
    alpha, beta, gamma = euler_angles  # 注意参数顺序！
    
    # 绕固定坐标系各轴的旋转矩阵
    Rz = np.array([  # 绕固定Z轴旋转gamma
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma),  np.cos(gamma), 0],
        [0,             0,             1]
    ])
    Ry = np.array([  # 绕固定Y轴旋转beta
        [np.cos(beta),  0, np.sin(beta)],
        [0,            1,            0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])
    Rx = np.array([  # 绕固定X轴旋转alpha
        [1, 0,            0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])
    
    # 固定坐标系旋转：Rx(α) * Ry(β) * Rz(γ)
    return Rx @ Ry @ Rz