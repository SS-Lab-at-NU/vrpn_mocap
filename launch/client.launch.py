import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    vrpn_config = LaunchConfiguration('vrpn_config').perform(context)
    server = LaunchConfiguration('server').perform(context)
    port = int(LaunchConfiguration('port').perform(context))

    debug_config = LogInfo(msg=[
        f'Using vrpn config file: {vrpn_config} with args: {server=} {port=}'
    ])

    # server/port defaults are declared in launch arguments, not config file
    vrpn_node = Node(
        package='vrpn_mocap',
        namespace='vrpn_mocap',
        executable='client_node',
        name='vrpn_mocap_client_node',
        parameters=[
            vrpn_config,
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
    vrpn_config_arg = DeclareLaunchArgument(
        'vrpn_config',
        default_value=os.path.join(share_dir, 'config', 'client.yaml'),
        description='Path to config file')

    server_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.0.226', # OptiTrack IP
        description='VRPN server address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='VRPN server port'
    )

    opaque_launch = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        vrpn_config_arg,
        server_arg,
        port_arg,
        opaque_launch
    ])
