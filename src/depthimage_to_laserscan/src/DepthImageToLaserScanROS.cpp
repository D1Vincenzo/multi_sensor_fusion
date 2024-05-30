#include <functional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp> // 引入ROS 2 C++客户端库的核心功能
#include <rclcpp_components/register_node_macro.hpp> // 用于注册节点组件
#include <sensor_msgs/msg/camera_info.hpp> // 引入相机信息消息类型
#include <sensor_msgs/msg/image.hpp> // 引入图像消息类型
#include <sensor_msgs/msg/laser_scan.hpp> // 引入激光扫描消息类型

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.hpp> // 引入深度图像转激光扫描的ROS封装

namespace depthimage_to_laserscan
{

DepthImageToLaserScanROS::DepthImageToLaserScanROS(const rclcpp::NodeOptions & options)
: rclcpp::Node("depthimage_to_laserscan", options) // 初始化节点
{
  auto qos = rclcpp::SystemDefaultsQoS(); // 获取系统默认的QoS设置
  // 订阅相机信息
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "depth_camera_info", qos, 
    std::bind(
      &DepthImageToLaserScanROS::infoCb, this, 
      std::placeholders::_1));

  // 订阅深度图像
  depth_image_sub_ =
    this->create_subscription<sensor_msgs::msg::Image>(
    "depth", qos,
    std::bind(&DepthImageToLaserScanROS::depthCb, this, std::placeholders::_1));

  // 发布激光扫描数据
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  // 声明并获取参数
  float scan_time = this->declare_parameter("scan_time", 0.033); // 扫描时间
  float range_min = this->declare_parameter("range_min", 0.45); // 最小范围
  float range_max = this->declare_parameter("range_max", 10.0); // 最大范围
  int scan_height = this->declare_parameter("scan_height", 1); // 扫描高度
  std::string output_frame = this->declare_parameter("output_frame", "camera_depth_frame"); // 输出帧ID

  // 初始化DepthImageToLaserScan对象
  dtl_ = std::make_unique<depthimage_to_laserscan::DepthImageToLaserScan>(
    scan_time, range_min, range_max, scan_height, output_frame);
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS() // 析构函数
{
}

// 处理相机信息回调
void DepthImageToLaserScanROS::infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  cam_info_ = info; // 保存相机信息
}

// 处理深度图像回调
void DepthImageToLaserScanROS::depthCb(const sensor_msgs::msg::Image::SharedPtr image)
{
  if (nullptr == cam_info_) {
    RCLCPP_INFO(get_logger(), "No camera info, skipping point cloud squash");
    return;
  }

  try {
    // 转换深度图像为激光扫描数据并发布
    sensor_msgs::msg::LaserScan::UniquePtr scan_msg = dtl_->convert_msg(image, cam_info_);
    scan_pub_->publish(std::move(scan_msg));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Could not convert depth image to laserscan: %s", e.what());
  }
}
}  // namespace depthimage_to_laserscan

// 注册这个节点为一个组件
RCLCPP_COMPONENTS_REGISTER_NODE(depthimage_to_laserscan::DepthImageToLaserScanROS)
