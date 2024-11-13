import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    config_file = LaunchConfiguration('config_file').perform(context)
    server = LaunchConfiguration('server').perform(context)
    port = LaunchConfiguration('port').perform(context)

    debug_config = LogInfo(msg=[
        f'Using vrpn config file: {config_file} with args: {server=} {port=}'
    ])

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

    return [
        debug_config,
        vrpn_node
    ]

def generate_launch_description():

    share_dir = get_package_share_directory('vrpn_mocap')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(share_dir, 'config', 'client.yaml'),
        description='Path to config file')

    server_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.0.2', # OptiTrack IP
        description='VRPN server address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='VRPN server port'
    )

    opaque_launch = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        config_file_arg,
        server_arg,
        port_arg,
        opaque_launch
    ])