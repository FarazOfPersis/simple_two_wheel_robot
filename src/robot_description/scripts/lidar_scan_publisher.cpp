#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarScanRepublisher : public rclcpp::Node
{
public:
    LidarScanRepublisher()
    : Node("lidar_scan_republisher")
    {
        // Subscriber: from Gazebo's simulated LiDAR
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/gz_lidar/scan",
            10,
            std::bind(&LidarScanRepublisher::scan_callback, this, std::placeholders::_1)
        );

        // Publisher: to /scan for RViz / navigation stack
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan",
            10
        );

        RCLCPP_INFO(this->get_logger(), "LiDAR scan republisher started: /gz_lidar/scan -> /scan");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Correct frame_id for RViz or TF tree
        msg->header.frame_id = "rplidar_c1";
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarScanRepublisher>());
    rclcpp::shutdown();
    return 0;
}
