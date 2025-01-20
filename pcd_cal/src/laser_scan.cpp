#include <limits>
#include <memory>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <rclcpp/clock.hpp>
#include "rosgraph_msgs/msg/clock.hpp"  // /clock 메시지 타입을 위해 추가

// auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_reliability_policy_e));

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher() : Node("laser_scan") {
    this->declare_parameter<std::string>("pcd_topic", "/gazebo_ros_laser_controller/out");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("imu_topic", "/imu_hpc/out");
    this->declare_parameter<float>("clipping_minz", -0.2);
    this->declare_parameter<float>("clipping_maxz", 0.4);
    pcd_topic = this->get_parameter("pcd_topic").as_string();
    scan_topic = this->get_parameter("scan_topic").as_string();
    imu_topic = this->get_parameter("imu_topic").as_string();
    clipping_minz = this->get_parameter("clipping_minz").as_double();
    clipping_maxz = this->get_parameter("clipping_maxz").as_double();

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // 신뢰할 수 있는 전송으로 설정
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // 노드 재시작 시 마지막 메시지 유지이

    publisher_scan = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);
    // publisher_scan1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_debug1", 10);
    // publisher_scan2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_debug2", 10);
    // publisher_scan3 = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_debug3", 10);

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pcd_topic, rclcpp::SensorDataQoS(), std::bind(&PCDPublisher::publishPointCloud, this, std::placeholders::_1));

    // imu_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   imu_topic, rclcpp::SensorDataQoS(), std::bind(&PCDPublisher::imuCallback, this, std::placeholders::_1));


    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), std::bind(&PCDPublisher::imuCallback, this, std::placeholders::_1));
    clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
  "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&PCDPublisher::clock_callback, this, std::placeholders::_1));

  }

private:
  void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(clock_mutex_);
    last_clock_ = msg->clock;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.push_back(*msg);
  }
  void publishPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_data_queue_.size() == 0){
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // sensor_msgs::msg::PointCloud2 output3;
    // pcl::toROSMsg(*cloud, output3);
    // output3.header.frame_id = "laser_data_frame";
    // output3.header.stamp = this->get_clock()->now();
    // publisher_scan3->publish(output3);

    Eigen::Matrix4f transform = get_tf_from_imu();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // sensor_msgs::msg::PointCloud2 output1;
    // pcl::toROSMsg(*transformed_cloud, output1);
    // output1.header.frame_id = "laser_data_frame";
    // output1.header.stamp = this->get_clock()->now();
    // publisher_scan1->publish(output1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    float height_tilted = std::cos(roll)*std::cos(pitch);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clipping_minz + (1.0 - height_tilted) * 0.6, clipping_maxz + (1.0 - height_tilted) * 0.6);  // minZ와 maxZ는 필터링할 Z값의 범위입니다.
    pass.filter(*cloud_filtered);

    // sensor_msgs::msg::PointCloud2 output2;
    // pcl::toROSMsg(*cloud_filtered, output2);
    // output2.header.frame_id = "laser_data_frame";
    // output2.header.stamp = this->get_clock()->now();
    // publisher_scan2->publish(output2);

    sensor_msgs::msg::LaserScan laser_scan = convertPointCloudToLaserScan(cloud_filtered);

    publisher_scan->publish(laser_scan);
  }

  sensor_msgs::msg::LaserScan convertPointCloudToLaserScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    sensor_msgs::msg::LaserScan scan;

    // LaserScan 메시지의 기본 설정
    scan.header.frame_id = "laser_data_frame";  // 프레임 ID 설정
    std::lock_guard<std::mutex> lock(clock_mutex_);
      if (last_clock_.nanosec > 0 || last_clock_.sec > 0) {  // 유효한 시간이 있는지 확인
        scan.header.stamp = last_clock_;
      } else {
        scan.header.stamp = this->get_clock()->now();  // /clock 데이터가 없는 경우 현재 시간 사용
      }
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180;  // 1도 간격 M_PI / 180.0
    scan.range_min = 0.5;
    scan.range_max = 200.0;  // 최대 거리 설정

    // 각도 범위에 따른 배열 크기 계산
    uint32_t ranges_size = static_cast<uint32_t>(std::round((scan.angle_max - scan.angle_min) / scan.angle_increment));
    scan.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    for (const auto& point : cloud->points) {
        float range = std::sqrt(point.x * point.x + point.y * point.y);
        if (range < scan.range_min || range > scan.range_max) {
            continue;  // 거리가 범위를 벗어나면 무시
        }

        float angle = std::atan2(point.y, point.x);
        if (angle < scan.angle_min || angle > scan.angle_max) {
            continue;  // 각도가 범위를 벗어나면 무시
        }

        // 각도를 인덱스로 변환
        int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);
        if (index >= 0 && index < static_cast<int>(ranges_size)) {
            scan.ranges[index] = std::min(range, scan.ranges[index]);  // 가장 가까운 거리 저장
        }
    }

    return scan;
  }

  Eigen::Matrix4f get_tf_from_imu() {
    // nav_msgs::msg::Odometry imu_data = imu_data_queue_.back();
    sensor_msgs::msg::Imu imu_data = imu_data_queue_.back();
    imu_data_queue_.clear();

    // Eigen::Quaternionf quaternion(imu_data.pose.pose.orientation.x, imu_data.pose.pose.orientation.y, 
    //                               imu_data.pose.pose.orientation.z, imu_data.pose.pose.orientation.w);
    // Eigen::Quaternionf quaternion(imu_data.orientation.x, imu_data.orientation.y, 
    //                               imu_data.orientation.z, imu_data.orientation.w);
    // Eigen::Matrix3f rotationMatrix = quaternion.toRotationMatrix();
    // Eigen::Vector3f euler_angles = rotationMatrix.eulerAngles(2, 1, 0);
    
    // euler_angles[2] = 0;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (imu_data.orientation.w * imu_data.orientation.x + imu_data.orientation.y * imu_data.orientation.z);
    float cosr_cosp = 1 - 2 * (imu_data.orientation.x * imu_data.orientation.x + imu_data.orientation.y * imu_data.orientation.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1 + 2 * (imu_data.orientation.w * imu_data.orientation.y - imu_data.orientation.x * imu_data.orientation.z));
    float cosp = std::sqrt(1 - 2 * (imu_data.orientation.w * imu_data.orientation.y - imu_data.orientation.x * imu_data.orientation.z));
    pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    // float siny_cosp = 2 * (imu_data.orientation.w * imu_data.orientation.z + imu_data.orientation.x * imu_data.orientation.y);
    // float cosy_cosp = 1 - 2 * (imu_data.orientation.y * imu_data.orientation.y + imu_data.orientation.z * imu_data.orientation.z);
    // yaw = std::atan2(siny_cosp, cosy_cosp);

    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    Eigen::Matrix3f modified_rotation_matrix;
    modified_rotation_matrix = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                          * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    // 변환을 위한 4x4 변환 행렬 생성, 여기서 yaw 회전은 무시됩니다.
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = modified_rotation_matrix;
    return transform ;
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_scan1;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_scan2;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_scan3;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // std::deque<nav_msgs::msg::Odometry> imu_data_queue_;
  std::deque<sensor_msgs::msg::Imu> imu_data_queue_;
  std::mutex mutex_;

  // 클리핑 값
  float clipping_minz;
  float clipping_maxz;
  std::string pcd_topic;
  std::string scan_topic;
  std::string imu_topic;

  float roll, pitch, yaw;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  std::mutex clock_mutex_;
  builtin_interfaces::msg::Time last_clock_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
