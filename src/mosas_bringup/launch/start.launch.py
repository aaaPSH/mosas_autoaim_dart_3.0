import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    camera_params_file = LaunchConfiguration(
        "camera_params_file",
        default=os.path.join(
            get_package_share_directory('mosas_bringup'),
            'config',
            'camera_params.yaml'
        )
    )
    detector_params_file = LaunchConfiguration(
        "detector_params_file",
        default=os.path.join(
            get_package_share_directory('mosas_bringup'),
            'config',
            'base_detect.yaml'
            # 'base_detect_test.yaml'
        )
    )
    serial_params_file = LaunchConfiguration(
        "serial_params_file",
        default=os.path.join(
            get_package_share_directory('mosas_bringup'),
            'config',
            'serial_params.yaml'
        )
    )

    container = ComposableNodeContainer(
        name="autoaim",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hik_publisher",
                plugin="HikCameraNode",
                name="hik_camera",
                parameters=[camera_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="detect_base",
                plugin="GreenDotDetectNode",
                name="green_dot_detect_node",
                parameters=[detector_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="can_serial",
                plugin="can_serial::CanSerialNode",
                name="can_serial_node",
                parameters=[serial_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # ComposableNode(
            #     package="save_frame",
            #     plugin="SaveFrameNode",
            #     name="SaveFrameNode",
            #     extra_arguments=[{"use_intra_process_comms": True}],
            # ),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_params_file", default_value=camera_params_file),
        DeclareLaunchArgument("detector_params_file", default_value=detector_params_file),
        container,
    ])
