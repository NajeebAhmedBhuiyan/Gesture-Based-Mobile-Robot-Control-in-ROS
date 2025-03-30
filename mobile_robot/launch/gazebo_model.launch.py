# Launch file for Differential Drive Robot in Gazebo with Arduino control

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Robot configuration
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'
    modelFileRelativePath = 'model/robot.xacro'
    pathModelFile = os.path.join(
        get_package_share_directory(namePackage),
        modelFileRelativePath
    )
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo launch
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )
    
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': '-r -v -v4 empty.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot state publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robotXacroName, '-topic', 'robot_description'],
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    gazeboBridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Keyboard teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix=['gnome-terminal --disable-factory --'],  # Open in a new terminal window
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    # Arduino bridge process
    arduino_script_path = os.path.join(
        get_package_share_directory(namePackage),
        'scripts',
        'arduino_bridge.py'
    )
    
    arduino_process = ExecuteProcess(
        cmd=['python3', arduino_script_path],
        output='screen',
        respawn=True,  # Auto-restart if crashes
        respawn_delay=3.0,  # Wait 3 seconds before restarting
        shell=True  # Helps with environment variables
    )

    return LaunchDescription([
        gazeboLaunch,
        nodeRobotStatePublisher,
        spawnModelNodeGazebo,
        gazeboBridge,
        teleop_node,
        arduino_process
    ])