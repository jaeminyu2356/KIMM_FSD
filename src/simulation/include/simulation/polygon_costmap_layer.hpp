#ifndef POLYGON_COSTMAP_LAYER_HPP
#define POLYGON_COSTMAP_LAYER_HPP

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>

namespace simulation
{

class PolygonCostmapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
    PolygonCostmapLayer();
    virtual ~PolygonCostmapLayer();

    virtual void onInitialize() override;
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double* min_x, double* min_y, double* max_x, double* max_y) override;
    virtual void updateCosts(
        nav2_costmap_2d::Costmap2D& master_grid,
        int min_i, int min_j, int max_i, int max_j) override;
    virtual void reset() override;
    virtual bool isClearable() override;

private:
    void polygonCallback(const geometry_msgs::msg::Polygon::SharedPtr msg);
    bool isPointInPolygon(double x, double y);

    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr polygon_sub_;
    std::vector<geometry_msgs::msg::Point32> polygon_points_;
    bool polygon_received_;
};

}  // namespace simulation

#endif  // POLYGON_COSTMAP_LAYER_HPP 