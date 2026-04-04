import os
import yaml
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from launch_ros.actions import Node


def prepend_namespace_to_yaml(namespace, yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    namespaced = {namespace: data}
    tmp = NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(namespaced, tmp)
    tmp.close()
    return tmp.name


def generate_launch_description():

    # Configuration variables
    package_name = 'autonomous_robot'
    namespace = 'arm'  #<--- CHANGE ME TO USE A DIFFERENT NAMESPACE

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # World configuration
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.sdf'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load in Gazebo'
    )

    # RViz configuration
    rviz_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'my_rviz.rviz'
    )

    # Include robot_state_publisher launch file with sim_mode=true
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Joystick for driving
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Pan-tilt joystick control
    pan_tilt_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pan_tilt_description'),
                'launch',
                'pan_tilt.joy.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'true'
        }.items()
    )

    # Twist mux for prioritizing cmd_vel sources
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', f'/{namespace}/diff_cont/cmd_vel_unstamped')]
    )

    # Twist stamper (note: joystick.launch.py also has one, may cause conflicts)
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in', f'/{namespace}/diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out', f'/{namespace}/diff_cont/cmd_vel')]
    )

    # Robot description with sim_mode=true
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "description",
        "robot.urdf.xacro"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro", " ", xacro_file,
            " use_ros2_control:=true",
            " sim_mode:=true"  # Important: enables Gazebo hardware interface
        ])
    }

    # Controller configuration with namespace
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
    namespaced_yaml = prepend_namespace_to_yaml(namespace, controller_params_file)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mini_pc_robot',
                   '-z', '0.1',
                   '-Y', '3.14159'],
        output='screen'
    )

    # Controller manager (with namespace)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description,
                    namespaced_yaml,
                    {'use_sim_time': True}],
        remappings=[(f'/{namespace}/robot_description', '/robot_description')],
        output="screen",
    )

    # Delay controller manager start
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Spawn diff_drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", f"/{namespace}/controller_manager"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Spawn joint_state_broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", f"/{namespace}/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Spawn pan_tilt_controller
    pan_tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_tilt_controller", "--controller-manager", f"/{namespace}/controller_manager"],
    )

    delayed_pan_tilt_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[pan_tilt_controller_spawner],
        )
    )

    # Gazebo-ROS bridge for sensors
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', f'config_file:={bridge_params}',
        ]
    )

    # Camera image bridge
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image_raw",
            '--ros-args',
            '-p', 'use_sim_time:=true'
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Launch everything
    return LaunchDescription([
        rviz_arg,
        world_arg,
        rsp,
        joystick,
        pan_tilt_joy_launch,
        twist_mux,
        twist_stamper,
        gazebo,
        spawn_entity,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_pan_tilt_controller_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        rviz,
    ])
