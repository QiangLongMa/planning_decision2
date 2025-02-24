from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
 
def generate_launch_description():
    node1 = Node(
            package="can",
            executable = "can_node",
            name= "can_node",
            output='screen'

            )
    node2 = Node(
            package="can",
            executable = "test1",
            name= "test",
         output='screen'
            )
    return LaunchDescription([node1,node2])
