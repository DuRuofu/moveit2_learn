import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 获取 robot_description 包的 share 目录路径
    robot_description_share_dir = get_package_share_directory('robot_description')

    # 拼接 URDF 文件路径
    urdf_path = os.path.join(robot_description_share_dir, 'urdf', 'robot.urdf.xacro')

    # 创建 launch description
    # 使用 Command + FindExecutable 来执行 xacro 并把输出作为 robot_description 参数
    robot_description_cmd = Command([FindExecutable(name='xacro'), ' ', urdf_path])

    return LaunchDescription([
        # 启动 robot_state_publisher 节点，传递 robot_description 参数（机器人的 URDF）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(robot_description_cmd, value_type=str)}]
        ),
        
        # 启动 joint_state_publisher_gui 节点，用于显示关节控制 GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # 启动 rviz2 节点，用于可视化机器人模型
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
