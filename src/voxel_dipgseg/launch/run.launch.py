# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():

    rviz_config = "src/demo.rviz"
   
    demo_node = Node(
        package="dipgseg",
        executable="demo_node",
        output='screen'
        )
  

    
    rviz_node = Node(
    package='rviz2',
    # node_executable='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
  )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([demo_node,rviz_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description


