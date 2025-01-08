#include <functional>
#include <time.h>
#include <errno.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong_msgs/msg/ping.hpp"
#include "pingpong_msgs/msg/pong.hpp"

#define S2NS 1000000000

class PongNode: public rclcpp::Node {
    rclcpp::Publisher<pingpong_msgs::msg::Pong>::SharedPtr pub_;
    rclcpp::Subscription<pingpong_msgs::msg::Ping>::SharedPtr sub_;
    void topic_callback_(const pingpong_msgs::msg::Ping::SharedPtr);
public:
    PongNode();
};

PongNode::PongNode(): Node("pong") {
    using namespace std::placeholders;
    pub_ = create_publisher<pingpong_msgs::msg::Pong>("pong", 10);
    sub_ = create_subscription<pingpong_msgs::msg::Ping>(
        "ping", 10, std::bind(&PongNode::topic_callback_, this, _1));
}

void PongNode::topic_callback_(const pingpong_msgs::msg::Ping::SharedPtr recv) {
    timespec time1, time2;
    pingpong_msgs::msg::Pong msg;

    clock_gettime(CLOCK_MONOTONIC, &time1);
    msg.t1_sec = (long)time1.tv_sec;
    msg.t1_nsec = (long)time1.tv_nsec;
    msg.t0_sec = recv->t0_sec;
    msg.t0_nsec = recv->t0_nsec;
    clock_gettime(CLOCK_MONOTONIC, &time2);
    msg.t2_sec = (long)time2.tv_sec;
    msg.t2_nsec = (long)time2.tv_nsec;
    pub_->publish(msg);
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
    auto pong_node = std::make_shared<PongNode>();

    struct sched_param param;
    param.sched_priority = 98;
    if(sched_setscheduler(0, SCHED_RR, &param) !=0) {
        perror("sched_setscheduler() failed");
        munlockall();
        return EXIT_FAILURE;
    }

    executor.add_node(pong_node);
    executor.spin();
    rclcpp::shutdown();
    munlockall();
}
