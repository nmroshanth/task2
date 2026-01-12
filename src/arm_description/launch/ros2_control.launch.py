from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Path to xacro
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("arm_description"),
            "urdf",
            "dual.urdf.xacro"
        ])
    ])

    robot_description = {
        "robot_description": robot_description_content
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("arm_description"),
        "config",
        "controllers.yaml"
    ])

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # ros2_control Controller Manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_yaml],
            output="screen",
        ),
        Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ])
