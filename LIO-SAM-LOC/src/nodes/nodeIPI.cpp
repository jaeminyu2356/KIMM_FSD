#include "ImuPreintegration.h"
#include "TransformFusion.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;

    auto ImuP = std::make_shared<IMUPreintegration>(options.arguments({"--ros-args", "-r", "__node:=ImuPreintegrationNode"}));
    auto TF = std::make_shared<TransformFusion>(options.arguments({"--ros-args", "-r", "__node:=TransformFusionNode"}));
    e.add_node(ImuP);
    e.add_node(TF);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> IMU Preintegration Started.\033[0m");

    e.spin();

    rclcpp::shutdown();
    return 0;
}