from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Drone 1 MAVROS Node (Joey)
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='Joey',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14555@'},
                {'gcs_url': 'udp://@127.0.0.1:14550'},
                {'tgt_system': 1},
                {'tgt_component': 1}
            ]
        ),
        
        # Drone 2 MAVROS Node (DeeDee)
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='DeeDee',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14565@'},
                {'gcs_url': 'udp://@127.0.0.1:14550'},
                {'tgt_system': 2},
                {'tgt_component': 1}
            ]
        ),
        
        # Drone 3 MAVROS Node (Marky)
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='Marky',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14575@'},
                {'gcs_url': 'udp://@127.0.0.1:14550'},
                {'tgt_system': 3},
                {'tgt_component': 1}
            ]
        )
    ])
