#include <memory>
#include <iostream>
#include <vector>
#include <limits>
#include <unordered_set>
#include <algorithm>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h>
#include <pcl/common/pca.h>

// #include <opencv2/opencv.hpp>

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher() : Node("pcd_publisher") {
    this->declare_parameter<std::string>("pcd_path", "/root/map_file/real_world_map/resolution0.8/GlobalMap.pcd");
    this->declare_parameter<float>("clipping_minz", -0.2); 
    this->declare_parameter<float>("clipping_maxz", 0.1);
    this->declare_parameter<int>("minNeighborsInRadius", 63);
    this->declare_parameter<float>("clustering_radius", 0.15);
    this->declare_parameter<float>("center_search_radius", 0.8);
    this->declare_parameter<float>("cluster_tolerance", 3.3);
    this->declare_parameter<float>("cos_theta_threshold", 0.9);
    this->declare_parameter<float>("dot2line_threshold", 1.0);
    this->declare_parameter<float>("dist_between_line", 3.0);
    this->declare_parameter<float>("linear_distribution_on_line", 1.5);
    this->declare_parameter<float>("grid_resolution", 0.05);
    this->declare_parameter<float>("occupancy_spread", 0.5);
    this->declare_parameter<bool>("same_length_flag", true); // 기본값을 true로 설정
    this->declare_parameter<std::string>("tree_pcd_path", "/root/map_file/tree_pcd/tree.pcd");
    this->declare_parameter<std::string>("yaml_filename", "/root/ros2_ws/src/kimm_orchard_sim/pcd_cal/maps/map.yaml");
    this->declare_parameter<std::string>("map_filename", "/root/ros2_ws/src/kimm_orchard_sim/pcd_cal/maps/map.pgm");
    
    pcd_path = this->get_parameter("pcd_path").as_string();
    clipping_minz = this->get_parameter("clipping_minz").as_double();
    clipping_maxz = this->get_parameter("clipping_maxz").as_double();
    minNeighborsInRadius = this->get_parameter("minNeighborsInRadius").as_int();
    clustering_radius = this->get_parameter("clustering_radius").as_double();
    center_search_radius = this->get_parameter("center_search_radius").as_double();
    cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    cos_theta_threshold = this->get_parameter("cos_theta_threshold").as_double();
    dot2line_threshold = this->get_parameter("dot2line_threshold").as_double();
    dist_between_line = this->get_parameter("dist_between_line").as_double();
    linear_distribution_on_line = this->get_parameter("linear_distribution_on_line").as_double();
    grid_resolution = this->get_parameter("grid_resolution").as_double();
    center_search_radius = this->get_parameter("center_search_radius").as_double();
    occupancy_spread = this->get_parameter("occupancy_spread").as_double();
    tree_pcd_path = this->get_parameter("tree_pcd_path").as_string();
    yaml_filename = this->get_parameter("yaml_filename").as_string();
    map_filename = this->get_parameter("map_filename").as_string();
    same_length_flag = this->get_parameter("same_length_flag").as_bool();

    RCLCPP_INFO(this->get_logger(), "clipping_minz: %f", clipping_minz);
    RCLCPP_INFO(this->get_logger(), "clipping_maxz: %f", clipping_maxz);
    RCLCPP_INFO(this->get_logger(), "minNeighborsInRadius: %d", minNeighborsInRadius);
    RCLCPP_INFO(this->get_logger(), "clustering_radius: %f", clustering_radius);
    RCLCPP_INFO(this->get_logger(), "center_search_radius: %f", center_search_radius);
    RCLCPP_INFO(this->get_logger(), "cluster_tolerance: %f", cluster_tolerance);
    RCLCPP_INFO(this->get_logger(), "cos_theta_threshold: %f", cos_theta_threshold);
    RCLCPP_INFO(this->get_logger(), "dot2line_threshold: %f", dot2line_threshold);
    RCLCPP_INFO(this->get_logger(), "dist_between_line: %f", dist_between_line);
    RCLCPP_INFO(this->get_logger(), "linear_distribution_on_line: %f", linear_distribution_on_line);
    RCLCPP_INFO(this->get_logger(), "grid_resolution: %f", grid_resolution);
    RCLCPP_INFO(this->get_logger(), "center_search_radius: %f", center_search_radius);
    RCLCPP_INFO(this->get_logger(), "occupancy_spread: %f", occupancy_spread);
    RCLCPP_INFO(this->get_logger(), "pcd_path: %s", pcd_path.c_str());
    RCLCPP_INFO(this->get_logger(), "tree_pcd_path: %s", tree_pcd_path.c_str());
    RCLCPP_INFO(this->get_logger(), "yaml_filename: %s", yaml_filename.c_str());
    RCLCPP_INFO(this->get_logger(), "map_filename: %s", map_filename.c_str());
    RCLCPP_INFO(this->get_logger(), "same_length_flag: %s", same_length_flag ? "true" : "false");

    publisher_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    publisher_pcd_filterd = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_filtered", 10);
    publisher_pcd_clipped = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_clipped", 10);
    publisher_pcd_coordinate = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_axis", 10);
    publisher_pcd_tree_center = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_tree_center", 10);
    publisher_pcd_tree_clustered = this->create_publisher<sensor_msgs::msg::PointCloud2>("publisher_pcd_tree_clustered", 10);
    publisher_pcd_tree_line = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_tree_line", 10);
    publisher_pcd_tree_grid = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_tree_grid", 10);
    publisher_pcd_tree_interpolation = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_tree_interpolation", 10);
    publisher_2d_occupancy_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&PCDPublisher::publishPointCloud, this));
  }

  struct linear_curve {
    pcl::PointXYZ point;
    Eigen::Vector3f eigen_vector;
  };

  int find_set(int element) {
    // 대표자를 찾습니다. 자기 자신이 대표자인 경우, 자기 자신을 반환합니다.
    if (element_to_set[element] == element) {
      return element;
    } else {
      // 경로 압축: 대표자를 찾는 과정에서 만나는 모든 요소의 대표자를 최종 대표자로 업데이트합니다.
      return element_to_set[element] = find_set(element_to_set[element]);
    }
  }

private:
  void publishPointCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_axis(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tree_center(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_tree_clustered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_tree_line(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_tree_grid(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load the PCD file.");
      return;
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clipping_minz, clipping_maxz);  // minZ와 maxZ는 필터링할 Z값의 범위입니다.
    pass.filter(*cloud_filtered);

    // pass.setInputCloud(cloud_filtered);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-100.0, 100.0);  // minZ와 maxZ는 필터링할 Z값의 범위입니다.
    // pass.filter(*cloud_filtered);

    // pass.setInputCloud(cloud_filtered);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-5.0, 0.0);  // minZ와 maxZ는 필터링할 Z값의 범위입니다.
    // pass.filter(*cloud_filtered);

    // RCLCPP_INFO(this->get_logger(), "PCD size : %ld", cloud_filtered -> size());

    // try {
    //     croped_pcd = cropPointCloud(cloud_filtered);
    // } catch (const std::exception& e) {
    //     RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in kdtree_search_radius: %s", e.what());
    //     return;
    // }

    try {
        cloud_radius = kdtree_search_radius(cloud_filtered);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in kdtree_search_radius: %s", e.what());
        return;
    }
    try {
        cloud_axis = find_point_cloud_coordinate();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in find_point_cloud_coordinate: %s", e.what());
        return;
    }
    try {
        cloud_tree_center = find_point_cloud_center(cloud_radius);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in find_point_cloud_center: %s", e.what());
        return;
    }
    try {
        point_cloud_tree_clustered = find_cluster(cloud_tree_center);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in find_cluster: %s", e.what());
        return;
    }
    try {
        point_cloud_tree_line = find_tree_line(point_cloud_tree_clustered);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in find_tree_line: %s", e.what());
        return;
    }
    try {
        point_cloud_tree_grid = find_grid(point_cloud_tree_line, cloud_tree_center, cloud_filtered);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in find_grid: %s", e.what());
        return;
    }
    try {
        saveOccupancyGridAsYAML(grid_2d, map_filename, yaml_filename);
        saveOccupancyGridAsPGM(grid_2d, map_filename);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_publisher"), "Exception caught in saving_occupancy_grid: %s", e.what());
        return;
    }
    // save_pcd(cloud_tree_center);

    sensor_msgs::msg::PointCloud2 output1;
    pcl::toROSMsg(*cloud, output1);
    output1.header.frame_id = "map";
    output1.header.stamp = this->get_clock()->now();
    publisher_pcd->publish(output1);

    sensor_msgs::msg::PointCloud2 output2;
    pcl::toROSMsg(*cloud_radius, output2);
    output2.header.frame_id = "map";
    output2.header.stamp = this->get_clock()->now();
    publisher_pcd_filterd->publish(output2);

    sensor_msgs::msg::PointCloud2 output3;
    pcl::toROSMsg(*cloud_filtered, output3);
    output3.header.frame_id = "map";
    output3.header.stamp = this->get_clock()->now();
    publisher_pcd_clipped->publish(output3);

    sensor_msgs::msg::PointCloud2 output4;
    pcl::toROSMsg(*cloud_axis, output4);
    output4.header.frame_id = "map";
    output4.header.stamp = this->get_clock()->now();
    publisher_pcd_coordinate->publish(output4);

    sensor_msgs::msg::PointCloud2 output5;
    pcl::toROSMsg(*cloud_tree_center, output5);
    output5.header.frame_id = "map";
    output5.header.stamp = this->get_clock()->now();
    publisher_pcd_tree_center->publish(output5);

    if (point_cloud_tree_clustered->size ()) {
      sensor_msgs::msg::PointCloud2 output6;
      pcl::toROSMsg(*point_cloud_tree_clustered, output6);
      output6.header.frame_id = "map";
      output6.header.stamp = this->get_clock()->now();
      publisher_pcd_tree_clustered->publish(output6);

      sensor_msgs::msg::PointCloud2 output7;
      pcl::toROSMsg(*point_cloud_tree_line, output7);
      output7.header.frame_id = "map";
      output7.header.stamp = this->get_clock()->now();
      publisher_pcd_tree_line->publish(output7);

      sensor_msgs::msg::PointCloud2 output8;
      pcl::toROSMsg(*point_cloud_tree_grid, output8);
      output8.header.frame_id = "map";
      output8.header.stamp = this->get_clock()->now();
      publisher_pcd_tree_grid->publish(output8);

      sensor_msgs::msg::PointCloud2 output9;
      pcl::toROSMsg(*point_interpolation_merged, output9);
      output9.header.frame_id = "map";
      output9.header.stamp = this->get_clock()->now();
      publisher_pcd_tree_interpolation->publish(output9);

      grid_2d.header.frame_id = "map";
      grid_2d.header.stamp = this->get_clock()->now();
      publisher_2d_occupancy_grid->publish(grid_2d);
    } 
    else {
      RCLCPP_INFO(this->get_logger(), "Clustering failed!!");
    }
  }

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cropPointCloud(
  //   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  //   const std::vector<Eigen::Vector3f>& rectangleCorners) {

  //   // 직사각형을 정의하는 네 점에서 최소 및 최대 x, y 좌표를 찾음
  //   float minX = std::min_element(rectangleCorners.begin(), rectangleCorners.end(),
  //                                 [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.x() < b.x(); })->x();
  //   float maxX = std::max_element(rectangleCorners.begin(), rectangleCorners.end(),
  //                                 [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.x() < b.x(); })->x();
  //   float minY = std::min_element(rectangleCorners.begin(), rectangleCorners.end(),
  //                                 [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.y() < b.y(); })->y();
  //   float maxY = std::max_element(rectangleCorners.begin(), rectangleCorners.end(),
  //                                 [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.y() < b.y(); })->y();

  //   // 필터링된 포인트 클라우드를 저장할 벡터
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr  filteredCloud;

  //   // 모든 점을 순회하면서 직사각형 영역에 포함되는지 확인
  //   for (const auto& point : cloud) {
  //     if (point.x() >= minX && point.x() <= maxX && point.y() >= minY && point.y() <= maxY) {
  //       filteredCloud -> push_back(point);
  //     }
  //   }

  //   return filteredCloud;
  // }

  pcl::PointCloud<pcl::PointXYZ>::Ptr kdtree_search_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // k-d 트리 객체를 생성합니다.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 필터링된 인덱스를 저장할 객체입니다.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // 모든 포인트에 대해 주변 이웃을 검사합니다.
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // 현재 포인트 주변의 이웃을 탐색합니다.
        if (kdtree.radiusSearch(cloud->points[i], clustering_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > minNeighborsInRadius) {
            // 최소 이웃 수 이상이면 결과에 포함시킵니다.
            inliers->indices.push_back(i);
        }
    }
    // 필터링된 포인트 클라우드를 추출합니다.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*filteredCloud);

    return filteredCloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr find_point_cloud_coordinate() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1, 0, 0));
    cloud->push_back(pcl::PointXYZ(0, 1, 0));
    cloud->push_back(pcl::PointXYZ(0, 0, 1));

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr find_point_cloud_center(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // k-d 트리 객체를 생성합니다.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 필터링된 인덱스를 저장할 객체입니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr central_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_set<int> processed_indices;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        // 이미 처리된 점은 건너뜁니다.
        if (processed_indices.find(i) != processed_indices.end()) {
            continue;
        }

        std::vector<int> point_indices;
        std::vector<float> point_squared_distance;

        // 반지름 내의 모든 점 검색
        if (kdtree.radiusSearch(cloud->points[i], center_search_radius, point_indices, point_squared_distance) > 0) {
            float sum_x = 0, sum_y = 0, sum_z = 0;
            int count = 0;

            // 원점으로부터 가장 가까운 점 찾기
            for (int idx : point_indices) {
                if (processed_indices.find(idx) != processed_indices.end()) {
                    continue; // 이미 처리된 점은 건너뜁니다.
                }
                sum_x += cloud->points[idx].x;
                sum_y += cloud->points[idx].y;
                sum_z += cloud->points[idx].z;
                count++;
                processed_indices.insert(idx);
            }
            if (count > 0) {
                pcl::PointXYZ central_point(sum_x / count, sum_y / count, sum_z / count);
                central_points->points.push_back(central_point);
            }
        }
    }
    return central_points;
  }

  void save_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::io::savePCDFile(tree_pcd_path, *cloud);
    return;
  }

  float squaredDistance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
    float diff_x = point2.x - point1.x;
    float diff_y = point2.y - point1.y;
    float diff_z = point2.z - point1.z;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr find_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tolerance); // 2cm
    ec.setMinClusterSize (3);
    ec.setMaxClusterSize (70);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

    int j = 0;
    for (const auto& cluster : cluster_indices) {
      for (const auto& idx : cluster.indices) {
        pcl::PointXYZI point;

        point.x = (*cloud)[idx].x;
        point.y = (*cloud)[idx].y;
        point.z = (*cloud)[idx].z;

        point.intensity = static_cast<float>(j);
        cloud_cluster->push_back(point);
      }
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      j++;
    }
    return cloud_cluster;
  }

  linear_curve pca_fitting_linear_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    for (auto& point : *cloud) {
      point.z = 0.0;
    }
    linear_curve curve_fittied;

    // RCLCPP_INFO(this->get_logger(), "%ld", cloud -> size());

    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);

    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    if((eigen_vectors.col(0)[0] < 0.0) && (eigen_vectors.col(0)[1] < 0.0)) {
      eigen_vectors.col(0)[0] = -eigen_vectors.col(0)[0];
      eigen_vectors.col(0)[1] = -eigen_vectors.col(0)[1];
    }

    if((eigen_vectors.col(0)[0] < 0.0) && (eigen_vectors.col(0)[1] > 0.0)) {
      eigen_vectors.col(0)[0] = -eigen_vectors.col(0)[0];
      eigen_vectors.col(0)[1] = -eigen_vectors.col(0)[1];
    }

    // RCLCPP_INFO(this->get_logger(), "eigen_vertor: %f, %f, %f",eigen_vectors.col(0)[0], eigen_vectors.col(0)[1], eigen_vectors.col(0)[2]);
    // RCLCPP_INFO(this->get_logger(), "point: %f, %f, %f",(*cloud)[0].x, (*cloud)[0].y, (*cloud)[0].z);
    
    curve_fittied.point.x = (*cloud)[1].x;
    curve_fittied.point.y = (*cloud)[1].y;
    curve_fittied.point.z = (*cloud)[1].z;
    curve_fittied.eigen_vector[0] = eigen_vectors.col(0)[0];
    curve_fittied.eigen_vector[1] = eigen_vectors.col(0)[1];
    curve_fittied.eigen_vector[2] = eigen_vectors.col(0)[2];

    return curve_fittied;
  }

  bool linear_curve_is_same(linear_curve a_line, linear_curve b_line){
    float dot_product = a_line.eigen_vector.dot(b_line.eigen_vector);
    float cos_theta = abs(dot_product / (a_line.eigen_vector.norm() * b_line.eigen_vector.norm()));

    // RCLCPP_INFO(this->get_logger(), "cos_theta: %f",cos_theta);

    if (cos_theta > cos_theta_threshold){
      Eigen::Vector3f A2B(a_line.point.x - b_line.point.x, a_line.point.y - b_line.point.y, a_line.point.z - b_line.point.z);
      Eigen::Vector3f crossProduct = A2B.cross(a_line.eigen_vector);

      float dist_between_dot2line = crossProduct.norm() / a_line.eigen_vector.norm();
      // RCLCPP_INFO(this->get_logger(), "dist_between_dot2line: %f",dist_between_dot2line);

      if (dist_between_dot2line < dot2line_threshold) {
        return true;
      }
    }
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr find_tree_line(pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud) {
    // ID를 키로 하고 포인트 클라우드를 값으로 하는 맵 정의
    // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudsById;
    cloudsById = std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr>();

    for (const auto& point : *clustered_cloud) {
      int id = static_cast<int>(point.intensity); 
      if (cloudsById.find(id) == cloudsById.end()) {
          cloudsById[id] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
      }
      cloudsById[id]->push_back(point);
    }

    std::vector<linear_curve> curves;

    for (long unsigned int i = 0; i < cloudsById.size(); i++) {
      curves.push_back(pca_fitting_linear_curve(cloudsById[i]));
    }

    std::unordered_map<int, std::vector<int>> id_candidates;

    for (long unsigned int i = 0; i < curves.size(); i++) {
      element_to_set[i] = i;
    }

    // 집합 병합
    for (long unsigned int i = 0; i < curves.size(); i++) {
      for (long unsigned int j = i + 1; j < curves.size(); j++) {
        // RCLCPP_INFO(this->get_logger(), "Id: %d_%d", i, j);
        if (linear_curve_is_same(curves[i], curves[j])) {
          element_to_set[find_set(j)] = find_set(i);  // j의 집합을 i의 집합과 병합
        }
      }
    }

    // 병합된 집합을 id_candidates에 추가
    for (long unsigned int i = 0; i < curves.size(); i++) {
      int set_id = find_set(i);
      id_candidates[set_id].push_back(i);
    }

    for (long unsigned int i = 0; i < curves.size(); i++) {
      if (id_candidates.find(i) != id_candidates.end()) {
        for (long unsigned int j : id_candidates[i]) {

          // RCLCPP_INFO(this->get_logger(), "Id: %d_%d", i, j);
          // RCLCPP_INFO(this->get_logger(), "bool: %d", (i == j));

          if (i == j)
            continue;

          if (cloudsById.find(j) != cloudsById.end()) {
            auto& cloud_j = cloudsById.find(j)->second;
            for (auto& point : *cloud_j) {
              point.intensity = static_cast<float>(i);
            }

          if (cloudsById.find(i) != cloudsById.end()) {
              cloudsById.find(i)->second->insert(cloudsById.find(i)->second->end(), cloud_j->begin(), cloud_j->end());
            }

            cloudsById.erase(cloudsById.find(j));
          }
        }
      }
    }

    for (const auto& pair : id_candidates) {
      std::string values_str;
      for (int val : pair.second) {
        values_str += std::to_string(val) + " ";
      }
      // RCLCPP_INFO(this->get_logger(), "Key: %d, Values: %s", pair.first, values_str.c_str());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pair : cloudsById) {
      // int id = pair.first; // 포인트 클라우드의 ID
      auto& cloud = pair.second;

      for (auto& point : *cloud) {
        mergedCloud->push_back(point);
      }
    }
    // RCLCPP_INFO(this->get_logger(), "-----------------------------");
    return mergedCloud;
  }

  bool is_left_or_right(linear_curve straight_line, pcl::PointXYZ point) {
    Eigen::Vector3f A2B= {point.x - straight_line.point.x, point.y - straight_line.point.y, point.z - straight_line.point.z};
    float cross_val = straight_line.eigen_vector.cross(A2B)[2];
    if (cross_val >= 0.0) {
      return true;
    }
    return false;
  }

  unsigned int positionToIndex(float x, float y, float resolution, float origin_x, float origin_y, int width) {
    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);
    return grid_x + grid_y * width;
  }

  void saveOccupancyGridAsYAML(const nav_msgs::msg::OccupancyGrid& grid, const std::string& map_filename, const std::string& yaml_filename) {
    std::ofstream out(yaml_filename);
    out << "image: " << map_filename << std::endl;
    out << "resolution: " << grid.info.resolution << std::endl;
    out << "origin: [" << grid.info.origin.position.x << ", " << grid.info.origin.position.y << ", " << grid.info.origin.position.z << "]\n";
    out << "occupied_thresh: 0.65\n";
    out << "free_thresh: 0.196\n";
    out << "negate: 0\n";
    out.close();
  }

  void saveOccupancyGridAsPGM(const nav_msgs::msg::OccupancyGrid& grid, const std::string& filename) {
    std::ofstream out(filename, std::ios::binary);
    out << "P5\n" << grid.info.width << " " << grid.info.height << "\n255\n";

    for (unsigned int y = 0; y < grid.info.height; y++) {
      for (unsigned int x = 0; x < grid.info.width; x++) {
        int index = x + y * grid.info.width;
        // ROS 2 OccupancyGrid에서 0은 자유, 100은 점유, -1(255로 표현)은 알 수 없음을 의미합니다.
        unsigned char value = (grid.data[index] == 0) ? 255 : (grid.data[index] == 100) ? 0 : 205;
        out.write(reinterpret_cast<const char*>(&value), sizeof(value));
      }
    }
    out.close();
  }

  Eigen::Vector3f rotateVector45Deg(Eigen::Vector3f vec) {
    float angle = M_PI / 4;  // 45 degrees in radians

    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << std::cos(angle), -std::sin(angle),
                      std::sin(angle),  std::cos(angle);

    Eigen::Vector2f vec2d(vec.x(), vec.y());
    Eigen::Vector2f rotatedVec2d = rotationMatrix * vec2d;

    return Eigen::Vector3f(rotatedVec2d.x(), rotatedVec2d.y(), vec.z());
  }

  bool findIntersection(linear_curve& line1, linear_curve& line2, pcl::PointXYZ& intersection) {
    Eigen::Matrix2d A;
    A << line1.eigen_vector.x(), -line2.eigen_vector.x(),
         line1.eigen_vector.y(), -line2.eigen_vector.y();
    
    if (std::abs(A.determinant()) < 1e-8) {
      // 두 직선이 평행하거나 일치합니다.
      return false;
    }

    Eigen::Vector2d b;
    b << line2.point.x - line1.point.x, line2.point.y - line1.point.y; 
    Eigen::Vector2d t = A.inverse() * b;

    Eigen::Vector2d answer = {line1.point.x + (t.x() * line1.eigen_vector)[0], line1.point.y + (t.x() * line1.eigen_vector)[1]};
    intersection.x = answer[0]; intersection.y = answer[1]; 
    return true;
  }

  pcl::PointXYZ moveInDirection(const pcl::PointXYZ& point, const Eigen::Vector3f& direction, float distance) {
    // 방향 벡터의 크기를 계산합니다.
    float magnitude = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    
    // 방향 벡터를 정규화합니다.
    Eigen::Vector3f normalizedDirection;
    normalizedDirection[0] = direction[0] / magnitude;
    normalizedDirection[1] = direction[1] / magnitude;
    normalizedDirection[2] = direction[2] / magnitude;  

    // 이동할 거리 벡터를 계산합니다.
    pcl::PointXYZ displacement;
    displacement.x = normalizedDirection[0] * distance;
    displacement.y = normalizedDirection[1] * distance;
    displacement.z = 0;  

    // 새로운 위치를 계산합니다.
    pcl::PointXYZ newPoint;
    newPoint.x = point.x + displacement.x;
    newPoint.y = point.y + displacement.y;
    newPoint.z = 0;  

    return newPoint;
  }

  void removeClosePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud, 
                         pcl::PointCloud<pcl::PointXYZI>::Ptr pointsToRemove, 
                         double threshold) {
  
  // for (auto& point : *pointsToRemove) {
    // RCLCPP_INFO(this->get_logger(),"points: %f,%f,%f", point.x, point.y, point.z);
  // }
  pcl::PointCloud<pcl::PointXYZ>::iterator it = originalCloud->begin();
    while (it != originalCloud->end()) {
      bool closePointFound = false;

      for (const auto& removePoint : *pointsToRemove) {
        pcl::PointXYZ point;
        point.x = removePoint.x;
        point.y = removePoint.y;
        point.z = removePoint.z;
        double distance = squaredDistance(*it, point);
        if (distance < threshold) {
          closePointFound = true;
          break;  // 가까운 포인트를 찾았으므로 더 이상 확인할 필요 없음
        }
      }

      if (closePointFound) {
        it = originalCloud->erase(it);  // 가까운 포인트 제거
      } else {
        ++it;
      }
    }
  } 

  bool comparePoints(const pcl::PointXYZ& a, const pcl::PointXYZ& b, const pcl::PointXYZ& standard) {
    float distA = std::pow(a.x - standard.x, 2) + std::pow(a.y - standard.y, 2) + std::pow(a.z - standard.z, 2);
    float distB = std::pow(b.x - standard.x, 2) + std::pow(b.y - standard.y, 2) + std::pow(b.z - standard.z, 2);
    return distA < distB;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_line(pcl::PointCloud<pcl::PointXYZ>::Ptr line_points) {
    pcl::PointXYZ certificate_x_small = {-10000, -10000, 0};

    std::sort(line_points->points.begin(), line_points->points.end(), [this, &certificate_x_small](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
      return this->comparePoints(a, b, certificate_x_small);
    });

    for (auto& point : *line_points) {
      // RCLCPP_INFO(this->get_logger(),"points:(x:%f,y:%f,z:%f)",point.x, point.y, point.z);
    }

    size_t point_cnt = line_points->points.size();
    size_t middle_index = point_cnt / 2;

    pcl::PointXYZ last_point = line_points->points[middle_index];

    // 중간 인덱스 이후 부분
    for (size_t i = middle_index + 1; i < point_cnt; i++) {
        double distance = squaredDistance(line_points->points[i], last_point);
        if (distance > 5 * linear_distribution_on_line) {
            line_points->points.erase(line_points->points.begin() + i, line_points->points.end());
            break;
        }
        last_point = line_points->points[i];
    }

    last_point = line_points->points[middle_index];
    // 중간 인덱스 이전 부분
    for (size_t i = middle_index; i > 0; i--) {
        double distance = squaredDistance(line_points->points[i], last_point);
        if (distance > 5 * linear_distribution_on_line) {
            line_points->points.erase(line_points->points.begin(), line_points->points.begin() + i + 1);
            break;
        }
        last_point = line_points->points[i];
    }

    return line_points;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointsBetween(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2, float distance) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    float total_distance = std::sqrt(std::pow(point2.x - point1.x, 2) + 
                                      std::pow(point2.y - point1.y, 2) + 
                                      std::pow(point2.z - point1.z, 2));

    int num_points = std::floor(total_distance / distance);

    float dir_x = (point2.x - point1.x) / total_distance;
    float dir_y = (point2.y - point1.y) / total_distance;
    float dir_z = (point2.z - point1.z) / total_distance;

    for (int i = 0; i <= num_points; ++i) {
      pcl::PointXYZ point;
      point.x = point1.x + dir_x * distance * i;
      point.y = point1.y + dir_y * distance * i;
      point.z = point1.z + dir_z * distance * i;
      cloud->points.push_back(point);
    }

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr find_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tree_points, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud) {
    int max_val = INT_MIN;
    int max_id = 0;

    if (cloudsById.empty()) {
      return 0;
    }
    //가장 많은 포인트를 가진 클러스터링 그룹 선택
    for (const auto& pair : cloudsById) {
      int id = pair.first; 
      auto& cloud_ = pair.second;
      // RCLCPP_INFO(this->get_logger(),"point_counts: %ld",cloud_ -> size());
      if (cloud_ -> size() > max_val) {
        max_id = id;
      }
    } 

    // RCLCPP_INFO(this->get_logger(), "a");
    //가장 많이 투표를 받은 그룹에 대한 PCA를 진행, 그 방향에 직각인 방향도 구함
    most_voted_curve = pca_fitting_linear_curve(cloudsById[max_id]);
    orthogonal_curve;
    orthogonal_curve.eigen_vector[0] = -most_voted_curve.eigen_vector[1];
    orthogonal_curve.eigen_vector[1] = most_voted_curve.eigen_vector[0];
    orthogonal_curve.eigen_vector[2] = 0.0;

    // RCLCPP_INFO(this->get_logger(), "b");
    // RCLCPP_INFO(this->get_logger(), "most_voted: %f, orthogonal: %f", (most_voted_curve.eigen_vector[1] / most_voted_curve.eigen_vector[0]), 
    //                                                                   (orthogonal_curve.eigen_vector[1] / orthogonal_curve.eigen_vector[0]));
    
    pcl::PointXYZ coordinate_origin_min = {FLT_MAX, FLT_MAX, 0.0};
    pcl::PointXYZ coordinate_origin_max = {-FLT_MAX, -FLT_MAX, 0.0};
    pcl::PointXYZ coordinate_origin_min_ = {-FLT_MAX, FLT_MAX, 0.0};
    pcl::PointXYZ coordinate_origin_max_ = {FLT_MAX, -FLT_MAX, 0.0};
    pcl::PointXYZ coordinate_origin_middle_for_certificate = {0.0, -FLT_MAX, 0.0};

    linear_curve finding_coordinate_curve_min; linear_curve finding_coordinate_curve_max;
    linear_curve finding_coordinate_curve_min_opposite; linear_curve finding_coordinate_curve_max_opposite;

    // finding_coordinate_curve_min.eigen_vector = rotateVector45Deg(most_voted_curve.eigen_vector);
    // finding_coordinate_curve_max.eigen_vector = rotateVector45Deg(most_voted_curve.eigen_vector);
    // finding_coordinate_curve_min_opposite.eigen_vector = rotateVector45Deg(orthogonal_curve.eigen_vector);
    // finding_coordinate_curve_max_opposite.eigen_vector = rotateVector45Deg(orthogonal_curve.eigen_vector);

    finding_coordinate_curve_min.eigen_vector = most_voted_curve.eigen_vector;
    finding_coordinate_curve_max.eigen_vector = most_voted_curve.eigen_vector;
    finding_coordinate_curve_min_opposite.eigen_vector = orthogonal_curve.eigen_vector;
    finding_coordinate_curve_max_opposite.eigen_vector = orthogonal_curve.eigen_vector;

    finding_coordinate_curve_min.point = coordinate_origin_min;
    finding_coordinate_curve_max.point = coordinate_origin_max;
    finding_coordinate_curve_min_opposite.point = coordinate_origin_min_;
    finding_coordinate_curve_max_opposite.point = coordinate_origin_max_;

    float pt_sum_x = 0.0;
    float pt_sum_y = 0.0;

    for (auto& point : *cloudsById[max_id]) {
      pt_sum_x = pt_sum_x + point.x;
      pt_sum_y = pt_sum_y + point.y;
    }

    //중앙점을 구해서 해당 점을 찾기 위한 출발점으로 지정
    float pt_average_x = pt_sum_x / cloudsById[max_id] -> size();
    float pt_average_y = pt_sum_y / cloudsById[max_id] -> size();

    coordinate_origin_middle_for_certificate.x = pt_average_x;
    coordinate_origin_middle_for_certificate.y = pt_average_y;

    // 이 부분이 애매한 부분, 중앙과 비교해서 curve가 올바른 방향인지 탐색함. 아니라면 반대 방향을 선택함
    if (!is_left_or_right(finding_coordinate_curve_min, coordinate_origin_middle_for_certificate)) {
      finding_coordinate_curve_min.eigen_vector = -finding_coordinate_curve_min.eigen_vector;
      finding_coordinate_curve_max.eigen_vector = finding_coordinate_curve_min.eigen_vector;
    }

    if (!is_left_or_right(finding_coordinate_curve_min_opposite, coordinate_origin_middle_for_certificate)) {
      finding_coordinate_curve_min_opposite.eigen_vector = -finding_coordinate_curve_min_opposite.eigen_vector;
      finding_coordinate_curve_max_opposite.eigen_vector = finding_coordinate_curve_min_opposite.eigen_vector;
    }

    //앞의 클러스터링 그룹들에 대해 각각을 하나의 직선으로 저장
    std::vector<std::pair<int, linear_curve>> curves;
    for (const auto& pair : cloudsById) {
      linear_curve curve = pca_fitting_linear_curve(pair.second);
      curves.push_back(std::make_pair(pair.first, curve));
    }

    for (auto& point : *tree_points) {
      point.z = 0;
    }

    //같은 eigen 성분을 가지는 포인트들에 한해서 limit 값을 구해서 사각형을 찾는다.
    for (const auto& curve_pair : curves) {
      int cloudId = curve_pair.first;  // 클라우드 ID
      const linear_curve& curve = curve_pair.second;

      // RCLCPP_INFO(this->get_logger(), "Curve ID: %d, eigen_vector: %f, %f, %f", cloudId, curve.eigen_vector[0], curve.eigen_vector[1], curve.eigen_vector[2]);
      // RCLCPP_INFO(this->get_logger(), "Curve point:%f, %f, %f", curve.point.x, curve.point.y, curve.point.z);

      float dot_product = curve.eigen_vector.dot(most_voted_curve.eigen_vector);
      float cos_theta = abs(dot_product / (curve.eigen_vector.norm() * most_voted_curve.eigen_vector.norm()));
      if (cos_theta < cos_theta_threshold) {
        if (cos_theta < cos_theta_threshold/4) {
          if (cloudsById.find(cloudId) != cloudsById.end()) {
            // RCLCPP_INFO(this->get_logger(), "cos_thetaeaeeae: %f", cos_theta);
            auto& cloud_ = cloudsById[cloudId];
            double threshold = 0.01;  // 가까운 포인트를 정의하는 임계값
            removeClosePoints(tree_points, cloud_, threshold);
          }
        }
        continue;
      }
      if (cloudsById.find(cloudId) != cloudsById.end()) {
        auto& cloud_ = cloudsById.find(cloudId)->second;
        for (auto& point : *cloud_) {
          pcl::PointXYZ pt(point.x, point.y, point.z);
          if (is_left_or_right(finding_coordinate_curve_min, pt)) {
            finding_coordinate_curve_min.point = pt;
          }
          if (!is_left_or_right(finding_coordinate_curve_max, pt)) {
            finding_coordinate_curve_max.point = pt;
          }
          if (is_left_or_right(finding_coordinate_curve_min_opposite, pt)) {
            finding_coordinate_curve_min_opposite.point = pt;
          }
          if (!is_left_or_right(finding_coordinate_curve_max_opposite, pt)) {
            finding_coordinate_curve_max_opposite.point = pt;
          }
        }
      }
    }

    save_pcd(tree_points);

    std::vector<linear_curve> lines = {finding_coordinate_curve_min, finding_coordinate_curve_max, finding_coordinate_curve_min_opposite, finding_coordinate_curve_max_opposite};
    std::vector<pcl::PointXYZ> intersections;

    // 모든 직선 쌍에 대해 교차점을 찾습니다.
    for (size_t i = 0; i < lines.size(); ++i) {
      for (size_t j = i + 1; j < lines.size(); ++j) {
        pcl::PointXYZ intersection;
        if (findIntersection(lines[i], lines[j], intersection)) {
          intersections.push_back(intersection);
        }
      }
    }

    // RCLCPP_INFO(this->get_logger(), "1st point:%f, %f, %f", intersections[0].x, intersections[0].y, intersections[0].z);
    // RCLCPP_INFO(this->get_logger(), "2st point:%f, %f, %f", intersections[1].x, intersections[1].y, intersections[1].z);
    // RCLCPP_INFO(this->get_logger(), "3st point:%f, %f, %f", intersections[2].x, intersections[2].y, intersections[2].z);
    // RCLCPP_INFO(this->get_logger(), "4st point:%f, %f, %f", intersections[3].x, intersections[3].y, intersections[3].z);

    std::sort(intersections.begin(), intersections.end(), 
      [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
      });

    // RCLCPP_INFO(this->get_logger(), "1st point:%f, %f, %f", intersections[0].x, intersections[0].y, intersections[0].z);
    // RCLCPP_INFO(this->get_logger(), "2st point:%f, %f, %f", intersections[1].x, intersections[1].y, intersections[1].z);
    // RCLCPP_INFO(this->get_logger(), "3st point:%f, %f, %f", intersections[2].x, intersections[2].y, intersections[2].z);
    // RCLCPP_INFO(this->get_logger(), "4st point:%f, %f, %f", intersections[3].x, intersections[3].y, intersections[3].z);

    //real
    // coordinate_origin_min = intersections[0]; //(x1, y1)
    // coordinate_origin_max = intersections[3]; // (x2, y2)
    // coordinate_origin_min_ = intersections[1]; // x좌표가 증가한 위치 (x2, y1)
    // coordinate_origin_max_ = intersections[2]; // y좌표가 증가한 위치 (x1, y2)

    //sim
    coordinate_origin_min = intersections[0]; //(x1, y1)
    coordinate_origin_max = intersections[3]; // (x1, y2)
    coordinate_origin_min_ = intersections[2]; // (x2, y1)
    coordinate_origin_max_ = intersections[1]; // (x2, y2)

    // float most_normalize = squaredDistance(coordinate_origin_min_, coordinate_origin_min);
    // float orthogonal_normalize = squaredDistance(coordinate_origin_max_, coordinate_origin_min);

    // most_voted_curve.eigen_vector[0] = (coordinate_origin_min_.x - coordinate_origin_min.x) / most_normalize;
    // most_voted_curve.eigen_vector[1] = (coordinate_origin_min_.y - coordinate_origin_min.y) / most_normalize;
    // most_voted_curve.eigen_vector[2] = (coordinate_origin_min_.z - coordinate_origin_min.z) / most_normalize;

    // orthogonal_curve.eigen_vector[0] = (coordinate_origin_max_.x - coordinate_origin_min.x) / orthogonal_normalize;
    // orthogonal_curve.eigen_vector[1] = (coordinate_origin_max_.y - coordinate_origin_min.y) / orthogonal_normalize;
    // orthogonal_curve.eigen_vector[2] = (coordinate_origin_max_.z - coordinate_origin_min.z) / orthogonal_normalize;

    // RCLCPP_INFO(this->get_logger(), "most_voted: %f, orthogonal: %f", (most_voted_curve.eigen_vector[1] / most_voted_curve.eigen_vector[0]), 
    //                                                                   (orthogonal_curve.eigen_vector[1] / orthogonal_curve.eigen_vector[0]));

    //최대 투표를 받은 커브에 대해서 해당 커브와 같은 벡터를 가진다고 판단이 되면 id 그룹에 추가해서 해당 선이 맞는 선으로 판단한다.
    std::vector<long unsigned int> rank_id;
    std::vector<float> rank_y_dist;

    int cnt = 0;

    // RCLCPP_INFO(this->get_logger(), "curves_count: %d", curves.size());
    for (const auto& curve_pair : curves) {
      int cloudId = curve_pair.first;  // 클라우드 ID
      const linear_curve& curve = curve_pair.second;

      float dot_product = curve.eigen_vector.dot(most_voted_curve.eigen_vector);
      float cos_theta = abs(dot_product / (curve.eigen_vector.norm() * most_voted_curve.eigen_vector.norm()));
      // RCLCPP_INFO(this->get_logger(), "cos_theta: %f", cos_theta);
      // RCLCPP_INFO(this->get_logger(), "theta: %f", std::acos(cos_theta) * 180 / M_PI);
      if (cos_theta < cos_theta_threshold) {
        
        continue;
      }
      // Eigen::Vector3f A2B(curves[i].point.x - finding_coordinate_curve_min.point.x, curves[i].point.y - finding_coordinate_curve_min.point.y, curves[i].point.z - finding_coordinate_curve_min.point.z);
      // Eigen::Vector3f A2B(curve.point.x - (coordinate_origin_min.x + coordinate_origin_min_.x) / 2.0, curve.point.y - (coordinate_origin_min.y + coordinate_origin_min_.y) / 2.0, 0);
      Eigen::Vector3f A2B(curve.point.x - (coordinate_origin_min.x + coordinate_origin_max_.x) / 2.0, curve.point.y - (coordinate_origin_min.y + coordinate_origin_max_.y) / 2.0, 0);
      Eigen::Vector3f crossProduct = A2B.cross(most_voted_curve.eigen_vector);

      float dist_between_dot2line = crossProduct.norm() / most_voted_curve.eigen_vector.norm();
      // RCLCPP_INFO(this->get_logger(), "dist: %f", dist_between_dot2line);

      rank_id.push_back(cnt);
      rank_y_dist.push_back(dist_between_dot2line);
      cnt++;
    }

    // 방금 만든 rank에 대해서 순서를 지정하기 위해 combined_key를 발행한다. 여기서 순서라는 것은 오른쪽이던, 왼쪽이던부터 사람이 인식할 수 있는 방향으로 라인의 순서를 지정하는 것이다.
    std::vector<std::pair<long unsigned int, float>> combined_key;
    for (size_t i = 0; i < rank_id.size(); ++i) {
        combined_key.emplace_back(rank_id[i], rank_y_dist[i]);
    }

    std::sort(combined_key.begin(), combined_key.end(), [](const auto& a, const auto& b) {
      return a.second < b.second;
    }); 

    // for (const auto& pair : combined_key) {
      // RCLCPP_INFO(this->get_logger(), "ID: %lu, Distance: %f", pair.first, pair.second);
    // }

    //새로운 커브를 형성한다. 이 커브들은 각 줄의 기준이 될 커브들이다.
    std::vector<linear_curve> new_curve_on_grid;
    linear_curve last_line;

    pcl::PointXYZ first_center;
    first_center.x = (coordinate_origin_min.x + coordinate_origin_max_.x) / 2;
    first_center.y = (coordinate_origin_min.y + coordinate_origin_max_.y) / 2;
    first_center.z = 0;

    last_line.point = first_center;

    bool first_flag = true;
    for (const auto& pair : combined_key) {
      linear_curve line;
      line.eigen_vector = most_voted_curve.eigen_vector;
      line.point = moveInDirection(first_center, orthogonal_curve.eigen_vector, pair.second);

      // RCLCPP_INFO(this->get_logger(), "point: %f,%f,%f", line.point.x, line.point.y, line.point.z);

      if ((squaredDistance(last_line.point, line.point) < (dist_between_line))) {
        if(first_flag) {
          first_flag = false;
        }
        else {
          continue;
        }
      }
      last_line = line;
      new_curve_on_grid.push_back(line);
    }

    // RCLCPP_INFO(this->get_logger(), "new_curve_on_grid: %d", new_curve_on_grid.size());
    // RCLCPP_INFO(this->get_logger(), "new_curve_on_grid point: %f, %f, %f", new_curve_on_grid[0].point.x, new_curve_on_grid[0].point.y, new_curve_on_grid[0].point.z);
    // RCLCPP_INFO(this->get_logger(), "new_curve_on_grid point: %f, %f, %f", new_curve_on_grid[1].point.x, new_curve_on_grid[1].point.y, new_curve_on_grid[1].point.z);
    // RCLCPP_INFO(this->get_logger(), "new_curve_on_grid point: %f, %f, %f", new_curve_on_grid[2].point.x, new_curve_on_grid[2].point.y, new_curve_on_grid[2].point.z);

    cloudsById_grid = std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto& point : *tree_points) {
      for (long unsigned int i = 0; i < new_curve_on_grid.size(); i++) {
        float min_dist = FLT_MAX;
        Eigen::Vector3f A2B(new_curve_on_grid[i].point.x - point.x, new_curve_on_grid[i].point.y - point.y, new_curve_on_grid[i].point.z - point.z);
        Eigen::Vector3f crossProduct = A2B.cross(new_curve_on_grid[i].eigen_vector);

        float dist_between_dot2line = crossProduct.norm() / new_curve_on_grid[i].eigen_vector.norm();
        // RCLCPP_INFO(this->get_logger(), "dist_between_dot2line: %f", dist_between_dot2line);
        
        if (min_dist > dist_between_dot2line) {
          min_dist = dist_between_dot2line;
          if (min_dist < linear_distribution_on_line) {            
            pcl::PointXYZI pt_id;
            pt_id.x = point.x;
            pt_id.y = point.y;
            pt_id.z = point.z;
            pt_id.intensity = static_cast<float>(i);
            grid_points -> push_back(pt_id);
            pcl::PointXYZ pt(point.x, point.y, point.z);
            if (cloudsById_grid.find(i) == cloudsById_grid.end()) {
              cloudsById_grid[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
            cloudsById_grid[i] -> push_back(pt);
          }
        }
      }
    }

    std::vector<int> keys_to_erase;

    for (auto& pair : cloudsById_grid) {
      // RCLCPP_INFO(this->get_logger(), "cloudsById_grid size: %ld", pair.second -> size());
      if (pair.second -> size() < 5) {
        keys_to_erase.push_back(pair.first);
      }
    }

    for (int key : keys_to_erase) {
      cloudsById_grid.erase(key);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (same_length_flag) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_interpolated(new pcl::PointCloud<pcl::PointXYZ>);
      float distance = squaredDistance(coordinate_origin_min, coordinate_origin_max_);
      int pts_cnt = distance / grid_resolution;

      // RCLCPP_INFO(this->get_logger(), "distance: %f,pts_cnt: %d", distance, pts_cnt);

      point_interpolation_merged.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointXYZ last_pt = coordinate_origin_min;
      for (int i = 0; i < pts_cnt; i++) {
        pcl::PointXYZ pt = moveInDirection(last_pt, most_voted_curve.eigen_vector, grid_resolution);
        point_interpolated -> push_back(pt);
        last_pt = pt;
      }

      cnt = 0;
      for (const auto& pair : combined_key) {
        // RCLCPP_INFO(this->get_logger(), "rank_y_dist: %f", pair.second);
        for (auto pt : *point_interpolated) {
          pcl::PointXYZ y_pt = moveInDirection(pt, orthogonal_curve.eigen_vector, pair.second);
          pcl::PointXYZI y_pt_i(y_pt.x, y_pt.y, y_pt.z, cnt);
          cnt++;
          point_interpolation_merged -> push_back(y_pt_i);
        }
      }
      for (auto& pair : cloudsById_grid) {
        auto& cloud_ = pair.second;
        cloud_ = filter_line(cloud_);
        for (auto& point : *cloud_) {
          pcl::PointXYZI pt;
          pt.x = point.x;
          pt.y = point.y;
          pt.z = point.z;
          pt.intensity = static_cast<float>(pair.first);

          mergedCloud->push_back(pt);
        }
      }
    } else {
      point_interpolation_merged.reset(new pcl::PointCloud<pcl::PointXYZI>);
      for (auto& pair : cloudsById_grid) {
        auto& cloud_ = pair.second;
        cloud_ = filter_line(cloud_);
        const pcl::PointXYZ& start_point = cloud_->points.front();
        const pcl::PointXYZ& end_point = cloud_->points.back();

        pcl::PointCloud<pcl::PointXYZ>::Ptr interpolated_line = generatePointsBetween(start_point, end_point, grid_resolution);
        pcl::PointCloud<pcl::PointXYZI>::Ptr interpolated_line_intensity(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto& point : *interpolated_line) {
          pcl::PointXYZI pt = {point.x, point.y, point.z, static_cast<float>(pair.first)};
          interpolated_line_intensity-> push_back(pt);
        }
        *point_interpolation_merged += *interpolated_line_intensity;
        for (auto& point : *cloud_) {
          pcl::PointXYZI pt(point.x, point.y, point.z, static_cast<float>(pair.first));
          mergedCloud->push_back(pt);
        }
      }
    }

    float width = 150.0;
    float height = 150.0;

    float padding_size = 1.0; // large than 1.0

    grid_2d.info.resolution = grid_resolution;  // 각 셀의 크기 (미터 단위)
    grid_2d.info.width = static_cast<int>(width / grid_resolution * padding_size);     // 맵의 너비 (셀의 수)
    grid_2d.info.height = static_cast<int>(height / grid_resolution * padding_size);      // 맵의 높이 (셀의 수)
    grid_2d.info.origin.position.x = -width / 2.0;  // 맵 원점의 x 좌표
    grid_2d.info.origin.position.y = -height / 2.0;  // 맵 원점의 y 좌표
    grid_2d.info.origin.position.z = 0.0;  // 맵 원점의 z 좌표
    grid_2d.info.origin.orientation.w = 1.0;

    // RCLCPP_INFO(this->get_logger(), "real_width: %f, real_height: %f", width, height);
    // RCLCPP_INFO(this->get_logger(), "width: %d, height: %d", grid_2d.info.width, grid_2d.info.height);
    // RCLCPP_INFO(this->get_logger(), "origin.position.x: %f, origin.position.y: %f", grid_2d.info.origin.position.x, grid_2d.info.origin.position.y);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_interpolation_merged_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    point_interpolation_merged_xyz->points.resize(point_interpolation_merged->points.size());

    for (size_t i = 0; i < point_interpolation_merged->points.size(); ++i) {
      point_interpolation_merged_xyz->points[i].x = point_interpolation_merged->points[i].x;
      point_interpolation_merged_xyz->points[i].y = point_interpolation_merged->points[i].y;
      point_interpolation_merged_xyz->points[i].z = point_interpolation_merged->points[i].z;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud(point_interpolation_merged);
    *point_interpolation_merged_xyz += *tree_points;
    kdtree.setInputCloud(point_interpolation_merged_xyz);

    grid_2d.data.resize(grid_2d.info.width * grid_2d.info.height);  // 데이터 배열 크기 설정
    int grid_spread = static_cast<int>(occupancy_spread / grid_2d.info.resolution);

    for (auto& point : *point_interpolation_merged_xyz) {
      int centerX = static_cast<int>((point.x - grid_2d.info.origin.position.x) / grid_2d.info.resolution);
      int centerY = static_cast<int>((point.y - grid_2d.info.origin.position.y) / grid_2d.info.resolution);
      unsigned int centerIndex = positionToIndex(point.x, point.y, grid_2d.info.resolution, grid_2d.info.origin.position.x, grid_2d.info.origin.position.y, grid_2d.info.width);

      grid_2d.data[centerIndex] = 100;  // 중앙 포인트 설정
      
      for (int y = -grid_spread; y <= grid_spread; y++) {
        for (int x = -grid_spread; x <= grid_spread; x++) {
          if (x * x + y * y <= grid_spread * grid_spread) {  // 원형 패턴 확인
            unsigned int newX = centerX + x;
            unsigned int newY = centerY + y;

            if (newX >= 0 && newX < grid_2d.info.width && newY >= 0 && newY < grid_2d.info.height) {
              unsigned int index = newX + newY * grid_2d.info.width;
              grid_2d.data[index] = 100;  // 장애물로 설정
            }
          }
        }
      }
    }

    return mergedCloud;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_filterd;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_clipped;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_coordinate;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_tree_center;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_tree_clustered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_tree_line;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_tree_grid;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_tree_interpolation;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_2d_occupancy_grid;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string pcd_path;
  std::string tree_pcd_path;
  std::string yaml_filename;
  std::string map_filename;
  float clipping_minz;
  float clipping_maxz;
  int minNeighborsInRadius;
  float clustering_radius;
  float center_search_radius;
  float cluster_tolerance;
  float cos_theta_threshold;
  float dot2line_threshold;
  float dist_between_line;
  float linear_distribution_on_line;
  float grid_resolution;
  float occupancy_spread;
  bool same_length_flag;

  std::unordered_map<int, int> element_to_set;
  std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudsById;
  std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudsById_grid;
  nav_msgs::msg::OccupancyGrid grid_2d;

  linear_curve most_voted_curve;
  linear_curve orthogonal_curve;

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_interpolation_merged;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}