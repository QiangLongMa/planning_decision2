from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():

    rviz_config = "src/demo.rviz"

    voxel_dipgseg = Node(
        package="dipgseg",
        executable="demo_node",
        output='screen'
    )
    lidar_processing = Node(
        package="lidar_processing",
        executable="Lidar_Processing_node",
        output='screen'
        )




    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    # executable='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
  )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([voxel_dipgseg,lidar_processing,rviz_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
