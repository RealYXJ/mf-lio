import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_lio_v45_cov_motion')
    config  = os.path.join(pkg_dir, 'config', 'robot_mid360_params.yaml')

    return LaunchDescription([
        Node(
            package='mf_lio_v45_cov_motion',
            executable='mf_lio_v45_cov_motion_node',
            name='mf_lio_v45_cov_motion_node',
            parameters=[config, {'use_sim_time': True}],
            output='screen',
        )
    ])
