#ifndef BUILD_QOS_H
#define BUILD_QOS_H

#include <rclcpp/rclcpp.hpp>

extern rmw_qos_profile_t qos_profile;
extern rclcpp::QoS qos;

extern rmw_qos_profile_t qos_profile_imu;
extern rclcpp::QoS qos_imu;

extern rmw_qos_profile_t qos_profile_lidar;
extern rclcpp::QoS qos_lidar;

#endif //BUILD_QOS_H
