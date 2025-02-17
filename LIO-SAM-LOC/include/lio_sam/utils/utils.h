//
// Created by root on 11/2/24.
//

#ifndef BUILD_UTILS_H
#define BUILD_UTILS_H

#include <rclcpp/rclcpp.hpp>

template<typename T>
double stamp2Sec(const T& stamp) {
    return rclcpp::Time(stamp).seconds();
}

#endif //BUILD_UTILS_H
