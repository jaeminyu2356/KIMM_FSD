#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <mutex>
#include <memory>

class PolygonPublisherNode : public rclcpp::Node
{
public:
    PolygonPublisherNode() : Node("polygon_publisher_node")
    {
        // 퍼블리셔 설정 - 큐 크기를 더 크게 설정하여 메시지 손실 방지
        polygon_pub_ = create_publisher<geometry_msgs::msg::Polygon>("costmap_polygon", 50);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("polygon_visualization", 50);
        
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
        
        // 시각화 메시지 초기화
        initializeVisualization();
        
        // 마커 업데이트 타이머 - 빠른 업데이트를 위해 20ms로 설정
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PolygonPublisherNode::timerCallback, this));
    }

private:
    void createPolygon()
    {
        std::vector<geometry_msgs::msg::Point> points = {
            makePoint(0.0, 0.0),
            makePoint(5.0, 0.0),
            makePoint(5.0, 5.0),
            makePoint(0.0, 5.0)
        };

        for (size_t i = 0; i < points.size(); ++i) {
            auto marker = createInteractiveMarker("point_" + std::to_string(i), points[i]);
            server_->insert(marker);
            server_->setCallback(marker.name,
                std::bind(&PolygonPublisherNode::processFeedback, this, std::placeholders::_1));
        }
        
        server_->applyChanges();
        needs_update_ = true;
    }

    void processFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            needs_update_ = true;
            last_updated_marker_ = feedback->marker_name;
        }
    }

    geometry_msgs::msg::Point makePoint(double x, double y)
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
        int_marker.scale = 0.3;  // 마커 크기 최적화

        // 마커의 시각적 표현
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // 시각적 컨트롤 (항상 표시)
        visualization_msgs::msg::InteractiveMarkerControl control_visual;
        control_visual.always_visible = true;
        control_visual.markers.push_back(marker);
        int_marker.controls.push_back(control_visual);

        // 이동 컨트롤 - 평면 이동만 허용
        visualization_msgs::msg::InteractiveMarkerControl control_move;
        control_move.name = "move_plane";
        control_move.orientation.w = 1;
        control_move.orientation.y = 1;
        control_move.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        control_move.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        int_marker.controls.push_back(control_move);

        return int_marker;
    }

    void initializeVisualization()
    {
        line_marker_.header.frame_id = "map";
        line_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker_.action = visualization_msgs::msg::Marker::ADD;
        line_marker_.scale.x = 0.03;
        line_marker_.color.g = 1.0;
        line_marker_.color.a = 1.0;
        line_marker_.pose.orientation.w = 1.0;
    }

    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(update_mutex_);
        if (needs_update_) {
            updatePolygonAndVisualization();
            needs_update_ = false;
        }
    }

    void updatePolygonAndVisualization()
    {
        geometry_msgs::msg::Polygon polygon_msg;
        line_marker_.points.clear();
        line_marker_.header.stamp = this->now();
        
        std::vector<geometry_msgs::msg::Point> points;
        points.reserve(4);  // 메모리 최적화

        for (int i = 0; i < 4; ++i) {
            visualization_msgs::msg::InteractiveMarker int_marker;
            if (server_->get("point_" + std::to_string(i), int_marker)) {
                // 폴리곤 메시지 업데이트
                geometry_msgs::msg::Point32 point;
                point.x = int_marker.pose.position.x;
                point.y = int_marker.pose.position.y;
                point.z = 0.0;
                polygon_msg.points.push_back(point);

                // 라인 마커 업데이트
                points.push_back(int_marker.pose.position);
            }
        }

        if (!points.empty()) {
            line_marker_.points = points;
            line_marker_.points.push_back(points[0]);  // 폴리곤 닫기
        }

        // 메시지 발행
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(line_marker_);
        
        polygon_pub_->publish(polygon_msg);
        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr polygon_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mutex update_mutex_;
    bool needs_update_{false};
    std::string last_updated_marker_;
    visualization_msgs::msg::Marker line_marker_;  // 재사용을 위해 멤버 변수로 저장
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolygonPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 