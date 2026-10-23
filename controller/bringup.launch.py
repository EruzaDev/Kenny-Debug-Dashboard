"""Keep SLAM, pose, policy, landmark detection and missions alive together."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    python = LaunchConfiguration('python')
    environment, model = LaunchConfiguration('environment'), LaunchConfiguration('model')
    slam_launch = Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('environment', description='Estimated or saved refined environment.json'),
        DeclareLaunchArgument('model', description='PPO checkpoint; loaded once'),
        DeclareLaunchArgument('python', default_value=str(root.parent / 'Kenny---Automotive-self-driving-trash-can/.venv/bin/python')),
        DeclareLaunchArgument('api', default_value='https://kenny-api.onrender.com'),
        DeclareLaunchArgument('motors', default_value='false', description='Run the USB encoder/motor bridge'),
        DeclareLaunchArgument('motor_port', default_value=''),
        DeclareLaunchArgument('markers', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true', description='Operations UI and required stop heartbeat on localhost:8002'),
        DeclareLaunchArgument('editor_bridge', default_value='true', description='Live refined map creator on localhost:8001'),
        DeclareLaunchArgument('auto_marker_start', default_value='false'),
        DeclareLaunchArgument('flat_floor', default_value='false'),
        DeclareLaunchArgument('commissioning', default_value='false'),
        DeclareLaunchArgument('drive_mode', default_value='velocity'),
        DeclareLaunchArgument('roam', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='0.1'),
        DeclareLaunchArgument('target_marker', default_value='-1'),
        DeclareLaunchArgument('initial_x', default_value='nan', description='Physical starting x in the estimated map'),
        DeclareLaunchArgument('initial_y', default_value='nan', description='Physical starting y in the estimated map'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam_launch)), launch_arguments={
            'slam_params_file': str(root / 'controller/slam.yaml'), 'use_sim_time': 'false',
            'autostart': 'true', 'use_lifecycle_manager': 'false'}.items()),
        ExecuteProcess(cmd=[python, '-m', 'controller.main', '--environment', environment, '--model', model,
            '--api', LaunchConfiguration('api'), '--commissioning', LaunchConfiguration('commissioning'),
            '--drive-mode', LaunchConfiguration('drive_mode'),
            '--roam', LaunchConfiguration('roam'),
            '--max-speed', LaunchConfiguration('max_speed'), '--target-marker', LaunchConfiguration('target_marker'),
            '--flat-floor', LaunchConfiguration('flat_floor'), '--auto-marker-start', LaunchConfiguration('auto_marker_start'), '--initial-pose', LaunchConfiguration('initial_x'),
            LaunchConfiguration('initial_y'), LaunchConfiguration('initial_yaw')], cwd=str(root), output='screen'),
        ExecuteProcess(cmd=[python, '-m', 'controller.markers', '--environment', environment],
                       cwd=str(root), output='screen', condition=IfCondition(LaunchConfiguration('markers'))),
        ExecuteProcess(cmd=[python, '-m', 'controller.motors', '--port', LaunchConfiguration('motor_port')],
                        cwd=str(root), output='screen', condition=IfCondition(LaunchConfiguration('motors'))),
        ExecuteProcess(cmd=[python, '-m', 'controller.dashboard'], cwd=str(root), output='screen',
                       condition=IfCondition(LaunchConfiguration('dashboard'))),
        ExecuteProcess(cmd=[python, str(root.parent / 'kenny-map-creator/ros/bridge.py'),
                            '--map-topic', '/kenny/map', '--map-frame', 'map', '--port', '8001'],
                       cwd=str(root), output='screen', condition=IfCondition(LaunchConfiguration('editor_bridge'))),
    ])
