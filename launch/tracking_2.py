from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_tracking',
            executable='car_tracker',
            name='car_tracker',
            remappings=[
                ('/image_raw/compressed', '/overhead_cam2/image_raw/compressed'),
                ('/debug_image', '/overhead_cam2/debug_image'),
            ],
            parameters=[
                {'id': 2}, 
            ],
        )
    ])