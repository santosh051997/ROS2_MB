import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from os import pathsep

def generate_launch_description():
    package_share_dir = get_package_share_directory("bumperbot_description")
    bumperbot_description_prefix = get_package_prefix("bumperbot_description")

    model_path = os.path.join(package_share_dir, "models")
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")

    world_file = os.path.join(package_share_dir, "worlds", "bumperbot_maze_camera.world")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(package_share_dir, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
     
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    
    env = {
        "GAZEBO_MODEL_PATH": model_path, # as we only to add maze_bot(model) into gazebo models path
    }

    return LaunchDescription(
        [
            model_arg,
            ExecuteProcess(
                cmd=["gazebo","--verbose",world_file,"-s","libgazebo_ros_factory.so",],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
        ]
    )