from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数, 由ros2 launch 扫描调用"""
    gnss_node = Node(
        package = 'gnss',  #package Name
        executable = 'gnss_node',  #executable file Name
        name='gnss_node',
        output='screen'
    )

    gnss_sync_node = Node(
        package = 'gnss',  #package Name
        executable = 'gnss_sync_node',  #executable file Name
        name='gnss_sync_node',
        output='screen'
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [gnss_node, gnss_sync_node]
    )
    #返回让ROS2根据launch描述执行节点
    return launch_description