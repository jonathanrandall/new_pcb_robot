from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to your robot URDF (adjust as needed)
    urdf_file = PathJoinSubstitution([
        FindPackageShare('esp32_combined_hardware'),
        'urdf',
        'esp32_robot.urdf.xacro'
    ])

    robot_description = Command(['xacro ', urdf_file])

    # Controller configuration (adjust paths as needed)
    controller_config = PathJoinSubstitution([
        FindPackageShare('esp32_combined_hardware'),
        'config',
        'controllers.yaml'
    ])

    # Joy button bridge configuration
    joy_button_config = PathJoinSubstitution([
        FindPackageShare('esp32_combined_hardware'),
        'config',
        'joy_button_mappings.yaml'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config
        ],
        output='screen'
    )

    # Spawn diff drive controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Spawn pan-tilt position controller
    pan_tilt_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_tilt_controller'],
        output='screen'
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Joy button bridge node
    joy_button_bridge = Node(
        package='esp32_combined_hardware',
        executable='joy_button_bridge',
        parameters=[joy_button_config],
        output='screen'
    )

    # Joy node (for joystick input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    # Teleop twist joy (to convert joystick to cmd_vel)
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 1.0,
            'enable_button': 4,  # L1 button
            'enable_turbo_button': 5  # R1 button
        }],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        controller_manager,
        diff_drive_spawner,
        pan_tilt_spawner,
        joint_state_broadcaster,
        joy_node,
        joy_button_bridge,
        teleop_twist_joy
    ])
