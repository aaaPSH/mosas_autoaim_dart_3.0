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
            # 'base_detect.yaml'
            'base_detect_test.yaml'
        )
    )
    system_monitor_params_file = LaunchConfiguration(
        "system_monitor_params_file",
        default=os.path.join(
            get_package_share_directory('mosas_bringup'),
            'config',
            'system_monitor_params.yaml'
        )
    )
    save_frame_params_file = LaunchConfiguration(
        "save_frame_params_file",
        default=os.path.join(
            get_package_share_directory('mosas_bringup'),
            'config',
            'save_frame.yaml'
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
                plugin="hik_publisher::HikCameraNode",
                name="hik_camera",
                parameters=[camera_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="detect_base",
                plugin="detect_base::GreenDotDetectNode",
                name="green_dot_detect_node",
                parameters=[detector_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="can_serial",
                plugin="can_serial::CanSerialNode",
                name="can_serial_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="save_frame",
                plugin="save_frame::SaveFrameNode",
                name="save_frame_node",
                parameters=[save_frame_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="system_monitor",
                plugin="system_monitor::SystemMonitorNode",
                name="system_monitor_node",
                parameters=[system_monitor_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_params_file", default_value=camera_params_file),
        DeclareLaunchArgument("detector_params_file", default_value=detector_params_file),
        DeclareLaunchArgument("system_monitor_params_file", default_value=system_monitor_params_file),
        DeclareLaunchArgument("save_frame_params_file", default_value=save_frame_params_file),
        container,
    ])
