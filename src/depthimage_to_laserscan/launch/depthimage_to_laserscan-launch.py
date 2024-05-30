import os
# 导入ament_index_python包，用于获取ROS2包的路径
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# 导入launch_ros中的Node类，用于定义节点
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数配置文件的路径
    # 使用get_package_share_directory函数找到'depthimage_to_laserscan'包的共享目录，
    # 然后连接子路径'cfg'和文件名'param.yaml'，以定位到参数配置文件。
    param_config = os.path.join(
        get_package_share_directory('depthimage_to_laserscan'), 'cfg', 'param.yaml')
    
    # 配置深度图像转激光扫描节点
    # 创建一个Node对象，配置它运行depthimage_to_laserscan包中的depthimage_to_laserscan_node可执行文件。
    # remappings参数用于重映射'depth'和'depth_camera_info'话题。
    # parameters参数用于加载前面获取的参数配置文件。
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[('depth', '/camera/depth/image_raw'),
                    ('depth_camera_info', '/camera/depth/camera_info'),
                    ('scan', '/dep2scan')],
        parameters=[param_config])

    # 配置静态变换发布节点
    # 创建另一个Node对象，配置它运行tf2_ros包中的static_transform_publisher可执行文件。
    # arguments参数用于指定静态变换的参数，包括平移和旋转参数，以及参考帧和目标帧的名称。
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laserscan',
        arguments=['0', '0.08', '0', '0', '-1.5708', '1.5708', 'camera_link_optical', 'frame']
        #arguments=['0', '0.08', '0', '0', '-1.5708', '1.5708', 'camera_link_optical', 'frame']
    )

    # 返回一个LaunchDescription对象
    # 包含了上面定义的两个节点，这个对象将在launch文件执行时启动这些节点。
    return LaunchDescription([
        depthimage_to_laserscan_node,
        static_tf_pub_node
    ])
