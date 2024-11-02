import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    server = LaunchConfiguration('server')
    server_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.0.2', # OptiTrack IP
        description='VRPN server address'
    )

    port = LaunchConfiguration('port')
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='VRPN server port'
    )

    share_dir = get_package_share_directory('vrpn_mocap')
    config_file = os.path.join(share_dir, 'config', 'client.yaml')

    # server/port launch arguments take priority over config file
    vrpn_node = Node(
        package='vrpn_mocap',
        namespace='vrpn_mocap',
        executable='client_node',
        name='vrpn_mocap_client_node',
        parameters=[
            config_file,
            {
                'server': server,
                'port': port
            }
        ]
    )

    return LaunchDescription([
        server_arg,
        port_arg,
        vrpn_node
    ])