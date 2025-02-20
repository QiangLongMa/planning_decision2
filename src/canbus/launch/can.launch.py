from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
 
def generate_launch_description():
    node1 = Node(
            package="canbus",
            executable = "canbus_node",
            name= "canbus_node",
            output='screen'

            )
    node2 = Node(
            package="canbus",
            executable = "mytest",
            name= "mytest",
         output='screen'
            )
    return LaunchDescription([node1,node2])
