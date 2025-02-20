# Based on OSRF's robot_localization examples


from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions

def generate_launch_description():


    director = os.path.dirname(os.path.abspath("whatever the path ends up being")) ## whatever the path ends up being
    repo_base = os.path.join(
        director,
        "dual_ekf_navsat_params.yaml"
    )
    
    # Path to EKF configuration file
    rl_params_file = os.path.join(director, 'dual_ekf_navsat_params.yaml')

    return LaunchDescription([
        
        ##launch.actions.DeclareLaunchArgument(
          ##  "use_sim_time", default_value="false",
          ##  description="Use simulation time if true"
        ##),

        # EKF node for odometry
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "odometry/local")],
        ),

        # EKF node for map
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),

        # NavSat transform node
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[
                ("imu", "imu"),
                ("gps/fix", "gps/fix"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/filtered", "odometry/global"),
            ],
        ),
    ])
