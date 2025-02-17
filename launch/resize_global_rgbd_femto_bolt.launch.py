import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    orbslam3_share_dir = get_package_share_directory('orbslam3')
    
    vocab_arg = DeclareLaunchArgument(
        'vocab',
        default_value=os.path.join(orbslam3_share_dir, 'vocabulary', 'ORBvoc.txt'),
        description='Path to vocabulary file'
    )
    
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(orbslam3_share_dir, 'config', 'rgb-d', 'Femto_Bolt_resized.yaml'),
        description='Path to config file'
    )
    
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='false',
        description='Enable visualization'
    )
    
    orbslam3_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orbslam3',
        output='screen',
        parameters=[os.path.join(orbslam3_share_dir, 'config', 'rgbd-slam-resized-global.yaml')],
        arguments=[
            LaunchConfiguration('vocab'),
            LaunchConfiguration('config'),
            LaunchConfiguration('visualization')
        ]
    )

    resize_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::ResizeNode',
            name='image_proc_node',
            parameters=[{
                'use_scale': True,
                'scale_height': 0.5,
                'scale_width': 0.5,
                'interpolation': 0,
            }],
            remappings=[
                ('/image', '/camera/color/image_raw'),
                ('/camera_info', '/camera/color/camera_info'),
                ('/resize/image_raw', '/camera/color/resize/image_raw'),
                ('/resize/camera_info', '/camera/color/resize/camera_info')
            ]
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::ResizeNode',
            name='image_proc_depth_node',
            parameters=[{
                'use_scale': True,
                'scale_height': 0.5,
                'scale_width': 0.5,
                'interpolation': 0,
            }],
            remappings=[
                ('/image', '/camera/depth/image_raw'),
                ('/camera_info', '/camera/depth/camera_info'),
                ('/resize/image_raw', '/camera/depth/resize/image_raw'),
                ('/resize/camera_info', '/camera/depth/resize/camera_info')
            ]
        )
    ]

    resize_container = ComposableNodeContainer(
        name='resize_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=resize_nodes,
        output='log'
    )


    return LaunchDescription([
        vocab_arg,
        config_arg,
        visualization_arg,
        orbslam3_node,
        resize_container
    ])
