from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_tracking',
            executable='car_tracker',
            name='car_tracker',
            remappings=[
                ('/image_raw/compressed', '/overhead_cam1/image_raw/compressed'),
                ('/debug/image_raw/compressed', '/overhead_cam1/debug/image_raw/compressed'),
            ],
            parameters=[
                {'id': 1}, 
                {'min_hsv_value': 180},
            ],
        )
    ])