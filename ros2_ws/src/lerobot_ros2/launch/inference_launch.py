import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lerobot_ros2')
    
    # 主配置文件路径
    robot_config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    
    # 动态获取 `supre_robot_joints.yaml` 的绝对路径
    joint_config_path = os.path.join(pkg_share, 'config', 'supre_robot_joints.yaml')
        
    return LaunchDescription([
        Node(
            package='lerobot_ros2',
            executable='policy_inference_server',
            name='policy_inference_server',
            output='screen',
            parameters=[
                robot_config_file,
                # 覆盖参数，将绝对路径传递给 robot.joint_config_path 字段
                # 这将被 RobotConfig.from_dict 解析并填充到 SupreRobotFollowerConfig 中
                {'robot.joint_config_path': joint_config_path}
            ]
        )
    ])