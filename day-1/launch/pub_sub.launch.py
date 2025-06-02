from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="day-1",
        executable="pub_cpp",
        name="pub_cpp",
    )

    subscriber_node = Node(
        package="day-1",
        executable="sub_cpp",
        name="sub_cpp",
    )

    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    
    return ld
