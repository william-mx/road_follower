from launch import LaunchDescription
from launch_ros.actions import Node

# Define the whitelist as a proper list
TOPIC_WHITELIST = [
    "/rc/ackermann_cmd",
    "/autonomous/ackermann_cmd",
    "/camera/camera/color/image_raw",
]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "topic_whitelist": TOPIC_WHITELIST
            }]
        ),

        Node(
            package="road_follower",
            executable="road_follower",
            name="road_follower",
            output="screen",
        ),
    ])
