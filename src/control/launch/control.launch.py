from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数, 由ros2 launch 扫描调用"""
    # control_node = Node(
    #     package = "control",  #package Name
    #     node_executable = "control_node",  #executable file Name
    #     output='screen'
    # )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        []
    )
    #返回让ROS2根据launch描述执行节点
    return launch_description
