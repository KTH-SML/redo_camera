//for udp
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <cmath>
//end for udp


#include "rclcpp/rclcpp.hpp"
#include "svea_vision_msgs/msg/person_state_array.hpp"
#include "mpc_guidance_bridge/ros2_receiver.hpp"

using svea_vision_msgs::msg::PersonStateArray;

MPCGuidance_Node::MPCGuidance_Node() 
: rclcpp::Node("mpc_guidance_node")
{
    //udp init
    udp_init_("127.0.0.1", 50051);

    auto qos = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<PersonStateArray>(
        "/person_state_estimation/person_states_kf", qos,
        // std::bind(&MPCGuidance_Node::print_test, this, std::placeholders::_1));
        std::bind(&MPCGuidance_Node::udp_test, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "MPC Guidance Node Started");
};

void MPCGuidance_Node::udp_init_(const std::string& host, uint16_t port){
    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if(udp_fd_ < 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
        udp_ready_ = false;
        return;
    }
    std::memset(&udp_dest_, 0, sizeof(udp_dest_));
    udp_dest_.sin_family = AF_INET;
    udp_dest_.sin_port = htons(port);
    if(::inet_pton(AF_INET, host.c_str(), &udp_dest_.sin_addr) <= 0){
        RCLCPP_ERROR(this->get_logger(), "Invalid address: %s", host.c_str());
        ::close(udp_fd_);
        udp_fd_ = -1;
        udp_ready_ = false;
        return;
    }
    udp_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "UDP socket initialized to %s:%d", host.c_str(), port);
}

void MPCGuidance_Node::print_test(const PersonStateArray::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received PersonStateArray with %zu states", msg->personstate.size());
    for (const auto& person : msg->personstate) {
        RCLCPP_INFO(this->get_logger(), "ID: %d Position: (%.2f, %.2f, %.2f)" "Velocity: (%.2f, %.2f) Acceleration: (%.2f, %.2f)", 
                    person.id, 
                    person.pose.position.x, 
                    person.pose.position.y, 
                    person.pose.position.z,
                    person.vx,
                    person.vy,
                    person.ax,
                    person.ay);
    }
};

bool MPCGuidance_Node::udp_send_xyvv_batch_(const std::vector<MPCGuidance_Node::XYVV>& elems) {
    if (!udp_ready_) return false;

    const size_t max_elems_per_packet = (MTU_SAFE - HEADER_SIZE) / ELEM_SIZE;
    size_t sent_total = 0;

    for (size_t off = 0; off < elems.size(); off += max_elems_per_packet) {
        const size_t chunk = std::min(max_elems_per_packet, elems.size() - off);

        std::vector<uint8_t> buf;
        buf.resize(HEADER_SIZE + chunk * ELEM_SIZE);

        //count
        uint32_t count_le = static_cast<uint32_t>(chunk);
        std::memcpy(buf.data(), &count_le, HEADER_SIZE);

        //content
        std::memcpy(buf.data() + HEADER_SIZE, elems.data() + off, chunk * ELEM_SIZE);

        ssize_t n = ::sendto(udp_fd_, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&udp_dest_),sizeof(udp_dest_));
        if (n != static_cast<ssize_t>(buf.size())){
            RCLCPP_ERROR(this->get_logger(), "UDP sendto failed: %s", std::strerror(errno));
            return false;
        }
        sent_total += chunk;
        RCLCPP_DEBUG(this->get_logger(), "Sent %zu elements in UDP packet", chunk);
    }
    return true;
}

void MPCGuidance_Node::udp_test(const PersonStateArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received PersonStateArray with %zu states", msg->personstate.size());
    
    std::vector<XYVV> packet;
    packet.reserve(msg->personstate.size());

    auto sane = [](float v){ return std::isfinite(v) ? v: 0.0f;};

    for (const auto& person : msg->personstate) {
        XYVV elem;
        elem.x = sane(static_cast<float>(person.pose.position.x));
        elem.y = sane(static_cast<float>(person.pose.position.y));
        elem.vx = sane(static_cast<float>(person.vx));
        elem.vy = sane(static_cast<float>(person.vy));
        packet.push_back(elem);
    }

    if (!packet.empty()){
        if (!udp_send_xyvv_batch_(packet)){
            RCLCPP_ERROR(this->get_logger(), "Failed to send UDP packet");
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "No data to send via UDP");
    }
}