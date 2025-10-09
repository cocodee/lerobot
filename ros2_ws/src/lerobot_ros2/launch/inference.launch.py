import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lerobot_ros2')
    
    # Launch文件只关心主配置文件
    config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
        
    return LaunchDescription([
        Node(
            package='lerobot_ros2',
            executable='policy_inference_server',
            name='policy_inference_server',
            output='screen',
            # 只传递一个配置文件
            parameters=[config_file]
        )
    ])