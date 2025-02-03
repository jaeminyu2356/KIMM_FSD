#include <iostream> 
#include <chrono> 
#include <memory> 

#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp> 

#include <pcl/point_cloud.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl_conversions/pcl_conversions.h> 

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher() : Node("pcd_publisher") {
    this->declare_parameter<std::string>("pcd_path", "/root/LOAM/GlobalMap.pcd");
    pcd_path = this->get_parameter("pcd_path").as_string();
    RCLCPP_INFO(this->get_logger(), "pcd_path: %s", pcd_path.c_str());

    publisher_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&PCDPublisher::publishPointCloud, this));
  }

private:
  void publishPointCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load the PCD file.");
      return;
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = this->get_clock()->now();
    publisher_pcd->publish(output);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string pcd_path;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
