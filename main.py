"""
UR5与CoppeliaSim联合仿真的主程序
"""
import numpy as np
import time
from kinematics import get_ur5_position, get_ur5_orientation, euler_to_matrix
from coppeliasim_remote import (
    connect_coppeliasim,
    get_joint_angles,
    set_joint_angles,
    get_tip_pose,
    disconnect_coppeliasim
)

def main():
    try:
        # 连接到CoppeliaSim
        connect_coppeliasim()
        print("成功连接到CoppeliaSim")
        while True:
            target_position = [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # 单位：弧度
            set_joint_angles(target_position)
            time.sleep(1)  # 等待运动完成
            # 获取当前关节角度
            joint_angles = get_joint_angles()
            
            # 获取末端位姿
            tip_pos, tip_ori = get_tip_pose()
            actual_R = euler_to_matrix(tip_ori)  # 将实际欧拉角转为旋转矩阵
            print(f"实际末端位置(m): X={tip_pos[0]:.3f}, Y={tip_pos[1]:.3f}, Z={tip_pos[2]:.3f}")
            print("实际旋转矩阵:\n", np.array2string(actual_R, precision=4, suppress_small=True))
        
            # 计算正运动学
            position = get_ur5_position(joint_angles)
            calculated_R = get_ur5_orientation(joint_angles)
            print(f"正运动学末端位置(m): X={position[0]:.3f}, Y={position[1]:.3f}, Z={position[2]:.3f}") 
            print("正运动学旋转矩阵:\n", np.array2string(calculated_R, precision=4, suppress_small=True))
            
                   
            # 示例控制逻辑
            # new_angles = [0.1, -0.5, 0.3, -1.2, -0.7, 0.5]
            # set_joint_angles(new_angles)
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n程序终止")
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        disconnect_coppeliasim()

if __name__ == "__main__":
    main()