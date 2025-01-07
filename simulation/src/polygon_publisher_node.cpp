#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

class PolygonPublisherNode : public rclcpp::Node
{
public:
    PolygonPublisherNode() : Node("polygon_publisher_node")
    {
        polygon_pub_ = create_publisher<geometry_msgs::msg::Polygon>("costmap_polygon", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("polygon_visualization", 1);
        
        // Interactive marker server 초기화
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "polygon_markers",
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_topics_interface(),
            this->get_node_services_interface());
        
        // 초기 폴리곤 생성
        createPolygon();
        
        // 마커 업데이트 타이머
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PolygonPublisherNode::timerCallback, this));
    }

private:
    void createPolygon()
    {
        // 초기 폴리곤 포인트 설정
        std::vector<geometry_msgs::msg::Point> points = {
            makePoint(0.0, 0.0, 0.0),
            makePoint(5.0, 0.0, 0.0),
            makePoint(5.0, 5.0, 0.0),
            makePoint(0.0, 5.0, 0.0)
        };

        // 각 점에 대해 interactive marker 생성
        for (size_t i = 0; i < points.size(); ++i) {
            auto marker = createInteractiveMarker("point_" + std::to_string(i), points[i]);
            server_->insert(marker, 
                std::bind(&PolygonPublisherNode::processFeedback, this, std::placeholders::_1));
        }
        
        server_->applyChanges();
    }

    void processFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        // 마커가 이동할 때마다 호출됨
        server_->applyChanges();
    }

    geometry_msgs::msg::Point makePoint(double x, double y, double z)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        return p;
    }

    visualization_msgs::msg::InteractiveMarker createInteractiveMarker(
        const std::string& name, const geometry_msgs::msg::Point& position)
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = name;
        int_marker.description = "Polygon Point";
        int_marker.pose.position = position;
        int_marker.scale = 1.0;
        
        // XY 평면 이동을 위한 컨트롤 설정
        visualization_msgs::msg::InteractiveMarkerControl control;
        
        // XY 평면에서만 이동하도록 설정
        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 1.0;  // Y축 방향으로 설정
        control.orientation.z = 0.0;
        
        control.name = "planar_control";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        control.always_visible = true;
        
        // 시각적 마커 추가
        visualization_msgs::msg::Marker marker = makeMarker();
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);
        
        return int_marker;
    }

    visualization_msgs::msg::Marker makeMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.1;  // z축 크기를 작게 설정
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        return marker;
    }

    void timerCallback()
    {
        publishPolygon();
        publishVisualization();
    }

    void publishPolygon()
    {
        geometry_msgs::msg::Polygon polygon_msg;
        
        for (int i = 0; i < 4; ++i) {
            visualization_msgs::msg::InteractiveMarker int_marker;
            if (server_->get("point_" + std::to_string(i), int_marker)) {
                geometry_msgs::msg::Point32 point;
                point.x = int_marker.pose.position.x;
                point.y = int_marker.pose.position.y;
                point.z = 0.0;
                polygon_msg.points.push_back(point);
            }
        }
        
        polygon_pub_->publish(polygon_msg);
    }

    void publishVisualization()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->now();
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.1;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        
        // 마커 포인트 수집
        std::vector<geometry_msgs::msg::Point> points;
        for (int i = 0; i < 4; ++i) {  // 4개의 포인트를 가정
            visualization_msgs::msg::InteractiveMarker int_marker;
            if (server_->get("point_" + std::to_string(i), int_marker)) {
                line_marker.points.push_back(int_marker.pose.position);
                points.push_back(int_marker.pose.position);
            }
        }
        
        // 폴리곤을 닫기 위해 첫 번째 점을 마지막에 추가
        if (!points.empty()) {
            line_marker.points.push_back(points.front());
        }
        
        marker_array.markers.push_back(line_marker);
        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr polygon_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolygonPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 