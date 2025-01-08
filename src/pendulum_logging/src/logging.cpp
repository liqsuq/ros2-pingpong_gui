#include <chrono>
#include <functional>
#include <time.h>
#include "rclcpp/rclcpp.hpp"
#include "pendulum_msgs/msg/rttest_results.hpp"

using namespace std;
using namespace pendulum_msgs::msg;

class LoggingNode: public rclcpp::Node
{
    rclcpp::Subscription<RttestResults>::SharedPtr sub_;
public:
    LoggingNode();
};

LoggingNode::LoggingNode(): Node("logging")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    auto callback = [](RttestResults::SharedPtr msg) {
        cout << msg->cur_latency << endl;
    };
    sub_ = this->create_subscription<RttestResults>(
        "pendulum_statistics", qos, callback);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<LoggingNode>());
    rclcpp::shutdown();
}
