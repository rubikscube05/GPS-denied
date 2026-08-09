from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Drone 1 MAVROS Node
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='Joy',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14550@14555'},
                {'tgt_system': 1},
                {'tgt_component': 1}
            ]
        ),
        
        # Drone 2 MAVROS Node
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='DeeDee',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14560@14565'},
                {'tgt_system': 2},
                {'tgt_component': 1}
            ]
        ),
        
        # Drone 3 MAVROS Node
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='Marky',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14570@14575'},
                {'tgt_system': 3},
                {'tgt_component': 1}
            ]
        )
    ])
