// 引入DepthImageToLaserScanROS类的定义
#include <depthimage_to_laserscan/DepthImageToLaserScanROS.hpp>
// 引入ROS 2 C++客户端库的核心功能
#include <rclcpp/rclcpp.hpp>

// 主函数
int main(int argc, char ** argv)
{
  // 初始化ROS 2节点
  rclcpp::init(argc, argv);

  // 创建DepthImageToLaserScanROS节点的实例
  // 使用make_shared来构建一个共享指针，指向新创建的DepthImageToLaserScanROS对象
  // rclcpp::NodeOptions()用于传递节点选项
  auto node = std::make_shared<depthimage_to_laserscan::DepthImageToLaserScanROS>(
    rclcpp::NodeOptions());

  // 进入循环，处理回调函数，直到ROS被关闭
  rclcpp::spin(node);

  // 关闭ROS，清理资源
  rclcpp::shutdown();

  // 返回0，表示程序正常结束
  return 0;
}
