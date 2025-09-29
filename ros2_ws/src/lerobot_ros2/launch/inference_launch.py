import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置文件路径
    config = os.path.join(
        get_package_share_directory('lerobot_ros2'),
        'config',
        'robot_config.yaml'
        )
        
    return LaunchDescription([
        Node(
            package='lerobot_ros2',
            executable='policy_inference_server',
            name='policy_inference_server',
            output='screen',
            parameters=[config]
        )
    ])