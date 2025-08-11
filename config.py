"""
UR5与CoppeliaSim联合仿真的配置参数
"""
from math import pi  # 添加这行导入语句

#Coppeliasim中UR5机械臂的改进DH参数 （alpha[i-1],a[i-1],d[i],theta_offset[i])
UR5_DH_PARAMS = [
    (0, 0, 0.06605, pi/2),
    (-pi/2, 0, 0.0703, -pi/2),
    (0, 0.4251, 0, 0),
    (0, 0.39215, 0.0397, pi/2),
    (pi/2, 0, 0.0952, pi),
    (pi/2, 0, 0.06225, 0)
]

# CoppeliaSim连接配置
COPPELIASIM_CONFIG = {
    'host': '127.0.0.1',
    'port': 19999,
    'joint_names': [
        'UR5_joint1', 'UR5_joint2', 'UR5_joint3',
        'UR5_joint4', 'UR5_joint5', 'UR5_joint6'
    ],
    'tip_name': 'Tip'
}