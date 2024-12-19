import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    orbslam3_share_dir = get_package_share_directory('orbslam3')
    
    vocab_arg = DeclareLaunchArgument(
        'vocab',
        default_value=os.path.join(orbslam3_share_dir, 'vocabulary', 'ORBvoc.txt'),
        description='Path to vocabulary file'
    )
    
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(orbslam3_share_dir, 'config', 'rgb-d', 'Femto_Bolt.yaml'),
        description='Path to config file'
    )
    
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='true',
        description='Enable visualization'
    )
    
    orbslam3_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orbslam3',
        output='screen',
        prefix=['gdbserver localhost:3000'],
        parameters=[os.path.join(orbslam3_share_dir, 'config', 'rgbd-slam-toro.yaml')],
        arguments=[
            LaunchConfiguration('vocab'),
            LaunchConfiguration('config'),
            LaunchConfiguration('visualization')
        ]
    )

    pose_tf_node = Node(
        package='toro_perception',
        executable='pose_tf.py',
        name='pose_tf',
        output='screen'
    )
    
    return LaunchDescription([
        vocab_arg,
        config_arg,
        visualization_arg,
        orbslam3_node,
        pose_tf_node
    ])
