from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    # Launch Gazebo Garden with custom world
    world_file = os.path.join(pkg_dir, 'worlds', 'robot_room.world')
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    
    # Convert xacro to URDF
    robot_description_content = Command(
        ['xacro ', urdf_file]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    
    # Bridge between Gazebo and ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_dir, 'config', 'bridge_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    # Spawn Robot using gz
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/robot_room/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '300',
             '--req', f'sdf_filename: "{urdf_file}", name: "robot", pose: {{position: {{x: 0, y: 0, z: 0.1}}}}'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        spawn_robot
    ]) 