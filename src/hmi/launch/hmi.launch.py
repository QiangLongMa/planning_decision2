from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数, 由ros2 launch 扫描调用"""
    hmi_node = Node(
        package = "hmi",  #package Name
        executable = "hmi",  #executable file Name
        output='screen'
    )
<<<<<<< HEAD
    local_trajs_node = Node(
        package = "local",  #package Name
        executable = "local_trajs_node",  #executable file Name
        output='screen'
    )
    gps_node = Node(
        package = "gps",  #package Name
        xecutable = "gps_node",  #executable file Name
        output='screen'
    )
=======
    planning_main_node = Node(
        package = "planning",  #package Name
        executable = "planning_main_node",  #executable file Name
        output='screen'
    )
    # gps_node = Node(
    #     package = "gps",  #package Name
    #     xecutable = "gps_node",  #executable file Name
    #     output='screen'
    # )
>>>>>>> 6a80d2887d19e67d9ce94b61132902778b175d56
    global_path_node = Node(
        package = "global",  #package Name
        executable = "global_path_node",  #executable file Name
        output='screen'
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
<<<<<<< HEAD
        [hmi_node, gps_node, global_path_node]
=======
        [hmi_node, planning_main_node, global_path_node]
>>>>>>> 6a80d2887d19e67d9ce94b61132902778b175d56
    )
    #返回让ROS2根据launch描述执行节点
    return launch_description
