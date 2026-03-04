#pragma once

#include "rclcpp/rclcpp.hpp"
#include "svea_vision_msgs/msg/person_state_array.hpp"

//for udp
#include <vector>
#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>


//end for udp


class MPCGuidance_Node : public rclcpp::Node {
    public:
        MPCGuidance_Node();
    private:
        void print_test(const svea_vision_msgs::msg::PersonStateArray::SharedPtr msg) const;
        rclcpp::Subscription<svea_vision_msgs::msg::PersonStateArray>::SharedPtr subscription_;
        struct XYVV {
            float x;
            float y;
            float vx;
            float vy;
        };
        static constexpr size_t MTU_SAFE = 1400;
        static constexpr size_t HEADER_SIZE = sizeof(uint32_t);
        static constexpr size_t ELEM_SIZE = sizeof(XYVV);

        static_assert(MPCGuidance_Node::ELEM_SIZE == 16, "Unexpected XYVV size");

        //for udp socket
        int udp_fd_{-1};
        sockaddr_in udp_dest_{};
        bool udp_ready_{false};

        void udp_init_(const std::string& host, uint16_t port);
        bool udp_send_xyvv_batch_(const std::vector<XYVV>& elems);
        void udp_test(const svea_vision_msgs::msg::PersonStateArray::SharedPtr msg);
        //end for udp
};
