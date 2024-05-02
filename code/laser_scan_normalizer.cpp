#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/LaserScan.hpp"

class LaserScanNormalizer : public rclcpp::Node
{
public:
  LaserScanNormalizer()
  : Node("laser_scan_normalizer")
  {
    // Adjust the topic names as necessary
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidarscan", 10, std::bind(&LaserScanNormalizer::laserscan_callback, this, std::placeholders::_1, 1));
    subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "dep2scan", 10, std::bind(&LaserScanNormalizer::laserscan_callback, this, std::placeholders::_1, 2));

    // Republish normalized LaserScan messages
    publisher1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("normalized_lidarscan", 10);
    publisher2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("normalized_dep2scan", 10);
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, int laser_id)
  {
    // Ensure the intensities field is not empty
    if (msg->intensities.empty())
    {
      // Example: Populate with zeros if empty. Adjust this logic based on your requirements.
      msg->intensities = std::vector<float>(360, 0.0);
    }

    // Republish the adjusted message on the corresponding topic
    if (laser_id == 1)
    {
      publisher1_->publish(*msg);
    }
    else if (laser_id == 2)
    {
      publisher2_->publish(*msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher1_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanNormalizer>());
  rclcpp::shutdown();
  return 0;
}
