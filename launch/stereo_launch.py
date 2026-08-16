from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='stereo_node'),
        DeclareLaunchArgument('voc_file', default_value=''),
        DeclareLaunchArgument('full_settings_path', default_value=''),
        DeclareLaunchArgument('img_left', default_value='/stereo_py_driver/img_left_msg'),
        DeclareLaunchArgument('img_right', default_value='/stereo_py_driver/img_right_msg'),

        Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[
                {'node_name_arg': LaunchConfiguration('node_name')},
                {'voc_file_arg': LaunchConfiguration('voc_file')},
                {'full_settings_path_arg': LaunchConfiguration('full_settings_path')},
                {'sub_img_left_topic': LaunchConfiguration('img_left')},
                {'sub_img_right_topic': LaunchConfiguration('img_right')},
            ]
        )
    ])
