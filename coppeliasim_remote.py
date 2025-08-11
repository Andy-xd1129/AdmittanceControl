"""
CoppeliaSim远程控制函数
"""
import sim
import numpy as np
from config import COPPELIASIM_CONFIG

# 全局连接状态
_client_id = -1
_joint_handles = []
_tip_handle = None

def connect_coppeliasim():
    """连接到CoppeliaSim"""
    global _client_id, _joint_handles, _tip_handle
    
    sim.simxFinish(-1)  # 关闭所有连接
    _client_id = sim.simxStart(
        COPPELIASIM_CONFIG['host'], 
        COPPELIASIM_CONFIG['port'], 
        True, True, 5000, 5
    )
    
    if _client_id == -1:
        raise ConnectionError("无法连接到CoppeliaSim")
    
    # 获取关节句柄
    _joint_handles = []
    for name in COPPELIASIM_CONFIG['joint_names']:
        _, handle = sim.simxGetObjectHandle(_client_id, name, sim.simx_opmode_blocking)
        _joint_handles.append(handle)
    # 获取末端执行器句柄
    res, _tip_handle = sim.simxGetObjectHandle(_client_id, COPPELIASIM_CONFIG['tip_name'], sim.simx_opmode_blocking)
    if res != sim.simx_return_ok:
        raise Exception("无法获取末端执行器句柄！请检查对象名称")
    return _client_id

def get_joint_angles():
    """获取当前关节角度"""
    global _client_id, _joint_handles
    angles = []
    for handle in _joint_handles:
        _, angle = sim.simxGetJointPosition(_client_id, handle, sim.simx_opmode_blocking)
        angles.append(angle)
    return angles

def set_joint_angles(angles):
    """设置关节目标角度"""
    global _client_id, _joint_handles
    for handle, angle in zip(_joint_handles, angles):
        sim.simxSetJointTargetPosition(_client_id, handle, angle, sim.simx_opmode_oneshot)

def get_tip_pose():
    """获取末端执行器位姿(位置和欧拉角)"""
    global _client_id, _tip_handle
    if _tip_handle is None:
        raise Exception("末端执行器句柄未初始化")
    
    # 获取位置
    _, tip_pos = sim.simxGetObjectPosition(_client_id, _tip_handle, -1, sim.simx_opmode_blocking)
    
    # 获取欧拉角
    _, tip_ori = sim.simxGetObjectOrientation(_client_id, _tip_handle, -1, sim.simx_opmode_blocking)
    
    return np.array(tip_pos), np.array(tip_ori)

def disconnect_coppeliasim():
    """断开连接"""
    global _client_id
    if _client_id != -1:
        sim.simxFinish(_client_id)
        _client_id = -1