# Launch file to display the robot in rviz2

import os  #to define directory, we can use library os
from ament_index_python.packages import get_package_share_directory #we can use this to avoid using full directory of file

from launch import LaunchDescription

# Generic actions can be imported from launch.actions module
from launch.actions import DeclareLaunchArgument  #import DeclareLaunchArgument to declare launcher parameters

# Substitutions allow you to get information from outside your launch file which is then substituted only when you execute it.
from launch.substitutions import Command, LaunchConfiguration #Import command class to change xacro format to plain urdf format
# to use the launch arguments we receive from command line,import Launchconfiguration class

# ROS-Specific actions come from ros_launch.actions module
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  #to declare a variable robot_description

# In this function, we will define all the componenets which we want to start and return a LaunchDescription object
def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    # Change behaviour of launch file based on input arguments
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bumperbot_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])