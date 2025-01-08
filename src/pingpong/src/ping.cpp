#include <chrono>
#include <functional>
#include <time.h>
#include <errno.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong_msgs/msg/ping.hpp"
#include "pingpong_msgs/msg/pong.hpp"
#include "pingpong_msgs/msg/logger.hpp"

#define S2NS 1000000000

class PingNode: public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pingpong_msgs::msg::Ping>::SharedPtr pub_ping_;
    rclcpp::Publisher<pingpong_msgs::msg::Logger>::SharedPtr pub_log_;
    rclcpp::Subscription<pingpong_msgs::msg::Pong>::SharedPtr sub_;
    void timer_callback_();
    void topic_callback_(pingpong_msgs::msg::Pong::SharedPtr);
public:
    PingNode();
};

PingNode::PingNode(): Node("ping") {
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    pub_ping_ = create_publisher<pingpong_msgs::msg::Ping>("ping", 10);
    timer_ = create_wall_timer(1ms, std::bind(&PingNode::timer_callback_, this));
    pub_log_ = create_publisher<pingpong_msgs::msg::Logger>("logger", 10);
    sub_ = create_subscription<pingpong_msgs::msg::Pong>(
        "pong", 10, std::bind(&PingNode::topic_callback_, this, _1));
}

void PingNode::timer_callback_() {
    timespec time0;
    pingpong_msgs::msg::Ping msg;

    clock_gettime(CLOCK_MONOTONIC, &time0);
    msg.t0_sec = (long)time0.tv_sec;
    msg.t0_nsec = (long)time0.tv_nsec;
    pub_ping_->publish(msg); 
}

void PingNode::topic_callback_(pingpong_msgs::msg::Pong::SharedPtr recv) {
    timespec time3;
    pingpong_msgs::msg::Logger msg;
    
    clock_gettime(CLOCK_MONOTONIC, &time3);
    msg.t0_sec = recv->t0_sec;
    msg.t0_nsec = recv->t0_nsec;
    msg.t1_sec = recv->t1_sec;
    msg.t1_nsec = recv->t1_nsec;
    msg.t2_sec = recv->t2_sec;
    msg.t2_nsec = recv->t2_nsec;
    msg.t3_sec = (long)time3.tv_sec;
    msg.t3_nsec = (long)time3.tv_nsec;
    pub_log_->publish(msg);
}

int main(int argc, char **argv) {
    // Memory locking
    if(mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall() failed");
        return EXIT_FAILURE;
    }
    // Memory allocate conig
    if(mallopt(M_TRIM_THRESHOLD, -1) != 1) {
        perror("mallopt(M_TRIM_THRESHOLD, -1) failed");
        munlockall();
        return EXIT_FAILURE;
    }
    if(mallopt(M_TOP_PAD, 512*1024) != 1) {
        perror("mallopt(M_TOP_PAD, ...) failed");
        munlockall();
        return EXIT_FAILURE;
    }
    if(mallopt(M_MMAP_MAX, 0) != 1) {
        perror("mallopt(M_MMAP_MAX, 0) failed");
        munlockall();
        return EXIT_FAILURE;
    }

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto ping_node = std::make_shared<PingNode>();

    struct sched_param param;
    param.sched_priority = 98;
    if(sched_setscheduler(0, SCHED_RR, &param) !=0) {
        perror("sched_setscheduler() failed");
        munlockall();
        return EXIT_FAILURE;
    }

    executor.add_node(ping_node);
    executor.spin();
    rclcpp::shutdown();
    munlockall();
}
