//
// Created by root on 11/2/24.
//

#ifndef BUILD_PCLUTILS_H
#define BUILD_PCLUTILS_H

#include "utils/pclType.h"
#include <sensor_msgs/msg/imu.hpp>


sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame);
float pointDistance(PointType p);
float pointDistance(PointType p1, PointType p2);


// it is come from MapOptimization
PointTypePose trans2PointTypePose(float transformIn[]);
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
Eigen::Affine3f trans2Affine3f(float transformIn[]);
void pointAssociateToMap(PointType const * const pi, PointType * const po, const Eigen::Affine3f& transPointAssociateToMap);

#endif //BUILD_PCLUTILS_H
