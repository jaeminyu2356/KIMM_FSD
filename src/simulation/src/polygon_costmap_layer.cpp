#include "simulation/polygon_costmap_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(simulation::PolygonCostmapLayer, nav2_costmap_2d::Layer)

namespace simulation
{

PolygonCostmapLayer::PolygonCostmapLayer()
: polygon_received_(false)
{
}

PolygonCostmapLayer::~PolygonCostmapLayer()
{
}

void PolygonCostmapLayer::onInitialize()
{
    current_ = true;
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock node");
    }

    polygon_sub_ = node->create_subscription<geometry_msgs::msg::Polygon>(
        "costmap_polygon", 1,
        std::bind(&PolygonCostmapLayer::polygonCallback, this, std::placeholders::_1));
}

void PolygonCostmapLayer::polygonCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    polygon_points_ = msg->points;
    polygon_received_ = true;
}

void PolygonCostmapLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!polygon_received_) {
        return;
    }

    for (const auto& point : polygon_points_) {
        *min_x = std::min(*min_x, static_cast<double>(point.x));
        *min_y = std::min(*min_y, static_cast<double>(point.y));
        *max_x = std::max(*max_x, static_cast<double>(point.x));
        *max_y = std::max(*max_y, static_cast<double>(point.y));
    }
}

void PolygonCostmapLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
    if (!polygon_received_) {
        return;
    }

    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);
            
            if (!isPointInPolygon(wx, wy)) {
                master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
            }
        }
    }
}

bool PolygonCostmapLayer::isPointInPolygon(double x, double y)
{
    bool inside = false;
    size_t i, j;
    for (i = 0, j = polygon_points_.size() - 1; i < polygon_points_.size(); j = i++) {
        if (((polygon_points_[i].y > y) != (polygon_points_[j].y > y)) &&
            (x < (polygon_points_[j].x - polygon_points_[i].x) * (y - polygon_points_[i].y) /
                    (polygon_points_[j].y - polygon_points_[i].y) +
                    polygon_points_[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

void PolygonCostmapLayer::reset()
{
    polygon_received_ = false;
    polygon_points_.clear();
}

bool PolygonCostmapLayer::isClearable()
{
    return true;
}

}  // namespace simulation 