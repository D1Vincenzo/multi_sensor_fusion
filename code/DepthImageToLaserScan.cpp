#include <depthimage_to_laserscan/DepthImageToLaserScan.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace depthimage_to_laserscan
{

/**
 * @brief 构造函数，用于创建 DepthImageToLaserScan 对象。
 * @param scan_time 扫描时间间隔。
 * @param range_min 最小扫描范围。
 * @param range_max 最大扫描范围。
 * @param scan_height 扫描高度。
 * @param frame_id 激光数据的坐标系标识。
 */
DepthImageToLaserScan::DepthImageToLaserScan(
  float scan_time, float range_min, float range_max,
  int scan_height, const std::string & frame_id)
: scan_time_(scan_time), range_min_(range_min), range_max_(range_max), scan_height_(scan_height),
  output_frame_id_(frame_id)
{
}

/*
这部分代码定义了`DepthImageToLaserScan`类的一些基本函数。

1. 构造函数：`DepthImageToLaserScan::DepthImageToLaserScan(...)`
  这是`DepthImageToLaserScan`类的构造函数，它接收几个参数，包括扫描时间、最小和最大范围、扫描高度和输出帧ID，
  并将这些参数保存在类的成员变量中。这些参数将用于后续的深度图像到激光扫描的转换。

2. 析构函数：`DepthImageToLaserScan::~DepthImageToLaserScan()`
  这是`DepthImageToLaserScan`类的析构函数，它在类的对象被销毁时被调用。在这个例子中，析构函数是空的，因为没有需要清理的资源。

3. `magnitude_of_ray`函数：`double DepthImageToLaserScan::magnitude_of_ray(const cv::Point3d & ray) const`
  这个函数计算一个射线的长度，它使用了三维空间中的距离公式。这个函数在计算两个射线之间的角度时会被用到。

4. `angle_between_rays`函数：`double DepthImageToLaserScan::angle_between_rays(...) const`
  这个函数计算两个射线之间的角度，它使用了向量的点积公式和`magnitude_of_ray`函数。这个函数在将深度图像转换为激光扫描时会被用到，
  因为需要计算每个像素点对应的射线与基准射线之间的角度，以确定激光扫描的角度。
*/ 
DepthImageToLaserScan::~DepthImageToLaserScan()
{
}
double DepthImageToLaserScan::magnitude_of_ray(const cv::Point3d & ray) const
{
  return std::sqrt(std::pow(ray.x, 2.0) + std::pow(ray.y, 2.0) + std::pow(ray.z, 2.0));
}

double DepthImageToLaserScan::angle_between_rays(
  const cv::Point3d & ray1,
  const cv::Point3d & ray2) const
{
  double dot_product = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;
  double magnitude1 = magnitude_of_ray(ray1);
  double magnitude2 = magnitude_of_ray(ray2);
  return std::acos(dot_product / (magnitude1 * magnitude2));
}

/**
 * 判断是否使用新的点
 * 
 * @param new_value 新的值
 * @param old_value 旧的值
 * @param range_min 最小范围
 * @param range_max 最大范围
 * @return 如果应该使用新的点，则返回true；否则返回false
 */
/*
这部分代码定义了DepthImageToLaserScan类中的use_point函数。这个函数的主要作用是决定是否使用新的深度值来替换旧的深度值。
这在将深度图像转换为激光扫描时非常重要，因为我们需要确定每个激光扫描点的深度值。

函数接收四个参数：新的深度值（new_value）、旧的深度值（old_value）、深度值的最小范围（range_min）和最大范围（range_max）。

函数首先检查新旧深度值是否是有限的（即不是NaN或Inf）。如果两者都不是有限的，那么它会选择不是NaN的值，因为Inf比NaN提供了更多的信息。

然后，函数检查新的深度值是否在指定的范围内。如果不在范围内，函数将返回false，表示不使用新的深度值。

如果旧的深度值不是有限的，但新的深度值是有限的并且在指定的范围内，函数将返回true，表示使用新的深度值。

最后，如果新旧深度值都是有限的，并且新的深度值比旧的深度值更接近，函数也会返回true，表示使用新的深度值。

总的来说，这个函数在确定每个激光扫描点的深度值时，优先选择有限的、在指定范围内的、更接近的深度值。
*/
bool DepthImageToLaserScan::use_point(
  const float new_value, const float old_value,
  const float range_min, const float range_max) const
{
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  bool new_finite = std::isfinite(new_value);
  bool old_finite = std::isfinite(old_value);

  // Infs are preferable over NaNs (more information)
  if (!new_finite && !old_finite) {  // Both are not NaN or Inf.
    return !std::isnan(new_value);  // new is not NaN, so use it's +-Inf value.
  }

  // If not in range, don't bother
  bool range_check = range_min <= new_value && new_value <= range_max;
  if (!range_check) {
    return false;
  }

  if (!old_finite) {  // New value is in range and finite, use it.
    return true;
  }

  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  bool shorter_check = new_value < old_value;
  return shorter_check;
}

/**
 * @brief 深度图像转激光扫描消息的转换函数
 * 
 * @param depth_msg 深度图像消息的共享指针
 * @param info_msg 相机信息消息的共享指针
 * @return sensor_msgs::msg::LaserScan::UniquePtr 激光扫描消息的唯一指针
 */
/*
这部分代码是`DepthImageToLaserScan`类中的`convert_msg`函数，它的主要作用是将深度图像转换为激光扫描。

函数接收两个参数：一个深度图像消息（`depth_msg`）和一个相机信息消息（`info_msg`）。

首先，函数使用相机信息消息设置相机模型（`cam_model_`）。

然后，函数计算激光扫描的最小角度（`angle_min`）和最大角度（`angle_max`）。这是通过测量左射线、右射线和光学中心射线之间的角度来完成的。

接着，函数创建一个新的激光扫描消息（`scan_msg`），并填充相关信息，如帧ID、最小角度、最大角度、角度增量、扫描时间、最小范围和最大范围。

然后，函数检查扫描高度（`scan_height_`）是否适合图像的高度。如果扫描高度过大，函数将抛出一个运行时错误。

接下来，函数根据深度图像的宽度计算并填充激光扫描的范围。

最后，函数根据深度图像的编码类型调用相应的`convert`函数进行转换。如果深度图像的编码类型不是16位无符号整数或32位浮点数，函数将抛出一个运行时错误。
*/
sensor_msgs::msg::LaserScan::UniquePtr DepthImageToLaserScan::convert_msg(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // 设置相机模型
  cam_model_.fromCameraInfo(info_msg);

  // 通过测量左射线、右射线和光学中心射线之间的角度来计算 angle_min 和 angle_max
  cv::Point2d raw_pixel_left(0, cam_model_.cy());
  cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

  cv::Point2d raw_pixel_right(depth_msg->width - 1, cam_model_.cy());
  cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

  double angle_max = angle_between_rays(left_ray, center_ray);
  // 负号是因为 laserscan 消息期望深度图像的相反旋转
  double angle_min = -angle_between_rays(center_ray, right_ray);

  // 填充 laserscan 消息
  sensor_msgs::msg::LaserScan::UniquePtr scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = depth_msg->header;
  if (output_frame_id_.length() > 0) {
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // 检查 scan_height 和 image_height
  if (static_cast<double>(scan_height_) / 2.0 > cam_model_.cy() ||
    static_cast<double>(scan_height_) / 2.0 > depth_msg->height - cam_model_.cy())
  {
    std::stringstream ss;
    ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
    throw std::runtime_error(ss.str());
  }

  // 计算并填充 ranges
  uint32_t ranges_size = depth_msg->width;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convert<uint16_t>(depth_msg, cam_model_, scan_msg, scan_height_);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convert<float>(depth_msg, cam_model_, scan_msg, scan_height_);
  } else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }

  return scan_msg;
}

}  // namespace depthimage_to_laserscan
