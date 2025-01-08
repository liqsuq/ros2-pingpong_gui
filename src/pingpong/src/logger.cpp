#include <chrono>
#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong_msgs/msg/ping.hpp"
#include "pingpong_msgs/msg/pong.hpp"
#include "pingpong_msgs/msg/logger.hpp"

#define S2NS 1000000000

class LoggerNode: public rclcpp::Node {
    rclcpp::Subscription<pingpong_msgs::msg::Logger>::SharedPtr sub_;
    void topic_callback_(pingpong_msgs::msg::Logger::SharedPtr);
public:
    LoggerNode();
};

LoggerNode::LoggerNode(): Node("logger") {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    sub_ = create_subscription<pingpong_msgs::msg::Logger>(
        "logger", 10, std::bind(&LoggerNode::topic_callback_, this, _1));
}

void LoggerNode::topic_callback_(pingpong_msgs::msg::Logger::SharedPtr msg) {
    long t0, t3, diff_t3t0;
    t0 = (long)(msg->t0_sec*S2NS + msg->t0_nsec);
    t3 = (long)(msg->t3_sec*S2NS + msg->t3_nsec);
    diff_t3t0 = (t3 - t0) / 1000;
    std::cout << diff_t3t0 << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto logger_node = std::make_shared<LoggerNode>();
    executor.add_node(logger_node);
    executor.spin();
    rclcpp::shutdown();
}
