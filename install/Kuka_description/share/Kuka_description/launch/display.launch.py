from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    Kuka_description = get_package_share_directory('Kuka_description')
    model_arg = DeclareLaunchArgument(name ='model',
                                      default_value=os.path.join(get_package_share_directory('Kuka_description'), 
                                                                 'urdf', 'kr150_2_macro.xacro'),
                                        description='Path to the KUKA robot model file')
    
    robot_desciption = ParameterValue(
        Command(['xacro ', model_arg]))
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desciption}],
        name='robot_state_publisher',
        output='screen',
        remappings=[
            ('/joint_states', '/kuka/joint_states')
        ]
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
    arguments=["-d", os.path.join(Kuka_description, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])