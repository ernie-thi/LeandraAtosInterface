from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer

def generate_launch_description():

    leandra_atos_interface_container = ComposableNodeContainer(
        name='leandra_atos_interface_container',
        namespace='leandra_atos_interface',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='leandra_atos_interface',
                plugin='LeandraAtosInterface::LeandraAtosInterfaceNode',
                name='leandra_atos_interface_node',
                namespace='leandra_atos_interface',
                parameters= [{
                    'ip_address': '127.0.0.1',   # default: localhost
                    #'ip_address': '192.168.2.51' # vehicle
                    }
                    ]
            ),
        ],
        output='screen',
        respawn=True,
        arguments= [
            "--ros-args",
            "--log-level",
            "INFO"
        ],
    )

    return LaunchDescription([
        leandra_atos_interface_container,
    ]) 
