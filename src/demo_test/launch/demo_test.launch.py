import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    publish_rate = LaunchConfiguration("publish_rate", default="160.0")
    detector_params_file = LaunchConfiguration(
        "detector_params_file",
        default=os.path.join(
            get_package_share_directory('mosas_bringup'),
            'config',
            'base_detect_test.yaml'
        )
    )

    demo_node = Node(
        package="demo_test",
        executable="demo_test",
        name="demo_test",
        parameters=[{
            "publish_rate": publish_rate,
        }],
        output="screen",
    )

    mock_mcu_node = Node(
        package="demo_test",
        executable="mock_mcu.py",
        name="mock_mcu",
        output="screen",
    )

    container = ComposableNodeContainer(
        name="demo_test_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="detect_base",
                plugin="detect_base::GreenDotDetectNode",
                name="green_dot_detect_node",
                parameters=[detector_params_file],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("publish_rate", default_value="160.0",
                              description="Image publish rate in Hz"),
        DeclareLaunchArgument("detector_params_file", default_value=detector_params_file),
        demo_node,
        mock_mcu_node,
        container,
    ])
