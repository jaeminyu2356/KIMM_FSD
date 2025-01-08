#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <memory>

class KimmCityNode : public rclcpp::Node
{
public:
    KimmCityNode() : Node("kimm_city_node")
    {
        // Gazebo 모델 상태를 구독
        model_state_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states",
            10,
            std::bind(&KimmCityNode::modelStateCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "KimmCity Node has been started");
    }

private:
    void modelStateCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // kimm_city 모델의 인덱스 찾기
        auto it = std::find(msg->name.begin(), msg->name.end(), "kimm_city");
        if (it != msg->name.end())
        {
            size_t index = std::distance(msg->name.begin(), it);
            RCLCPP_INFO(this->get_logger(), 
                "KIMM City position: x=%.2f, y=%.2f, z=%.2f",
                msg->pose[index].position.x,
                msg->pose[index].position.y,
                msg->pose[index].position.z);
        }
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KimmCityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 