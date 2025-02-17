//
// Created by root on 11/2/24.
//

#ifndef BUILD_TRANSFORMFUSION_H
#define BUILD_TRANSFORMFUSION_H

#include "utils/ParamServer.h"
#include "utils/utils.h"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>

class TransformFusion : public ParamServer {
        public:
        std::mutex mtx;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

        rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
        rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

        Eigen::Isometry3d lidarOdomAffine;
        Eigen::Isometry3d imuOdomAffineFront;
        Eigen::Isometry3d imuOdomAffineBack;

        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        tf2::Stamped<tf2::Transform> lidar2Baselink;

        double lidarOdomTime = -1;
        deque<nav_msgs::msg::Odometry> imuOdomQueue;

        TransformFusion(const rclcpp::NodeOptions & options);
        Eigen::Isometry3d odom2affine(nav_msgs::msg::Odometry odom);
        void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
        void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
};


#endif //BUILD_TRANSFORMFUSION_H
