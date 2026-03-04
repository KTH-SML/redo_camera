#include "rclcpp/rclcpp.hpp"
#include "mpc_guidance_bridge/ros2_receiver.hpp"

int main(){
    rclcpp::init(0, nullptr);
    auto kf_sub_node = std::make_shared<MPCGuidance_Node>();
    rclcpp::spin(kf_sub_node);
    rclcpp::shutdown();
    return 0;
}