from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='stereo_node'),
        DeclareLaunchArgument('voc_file', default_value=''),
        DeclareLaunchArgument('settings_path', default_value=''),
        DeclareLaunchArgument('sub_config', default_value='/stereo_py_driver/experiment_settings'),
        DeclareLaunchArgument('pub_ack', default_value='/stereo_py_driver/exp_settings_ack'),
        DeclareLaunchArgument('img_left', default_value='/stereo_py_driver/img_left_msg'),
        DeclareLaunchArgument('img_right', default_value='/stereo_py_driver/img_right_msg'),
        DeclareLaunchArgument('timestep', default_value='/stereo_py_driver/timestep_msg'),

        Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[
                {'node_name_arg': LaunchConfiguration('node_name')},
                {'voc_file_arg': LaunchConfiguration('voc_file')},
                {'settings_file_path_arg': LaunchConfiguration('settings_path')},
                {'sub_experiment_config_topic': LaunchConfiguration('sub_config')},
                {'pub_config_ack_topic': LaunchConfiguration('pub_ack')},
                {'sub_img_left_topic': LaunchConfiguration('img_left')},
                {'sub_img_right_topic': LaunchConfiguration('img_right')},
                {'sub_timestep_topic': LaunchConfiguration('timestep')},
            ]
        )
    ])
