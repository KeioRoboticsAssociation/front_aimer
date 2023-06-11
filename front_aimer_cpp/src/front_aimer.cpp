// detect circle from laser_scan
#include <chrono>
#include <math.hpp>
#include <memory>
#include <vector>

#include "front_aimer_interfaces/msg/target.hpp"
#include "front_aimer_interfaces/srv/target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class FrontAimer : public rclcpp::Node {
 public:
  FrontAimer() : Node("front_aimer") {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&FrontAimer::laser_callback, this, _1));
    target_pub_ = this->create_publisher<front_aimer_interfaces::msg::Target>(
        "target", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&FrontAimer::timer_callback, this));
  }

 private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 正面から±10度で、距離が8m以下の点を抽出
    float angle, X, Y;
    for (int i = 0; i < sizeof(msg->ranges) / sizeof(float); i++) {
      angle = (msg->angle_min + i * msg->angle_increment) * 180 / M_PI;
        if (angle > -5 && angle < 5 && msg->ranges[i] < 8) {
            X = msg->ranges[i] * cos(angle);
            Y = msg->ranges[i] * sin(angle);
            points.push_back(std::make_pair(X, Y));
        }
    }
  }
  void timer_callback() {
    front_aimer_interfaces::msg::Target target_msg;
    target_msg.angle = 0.0;
    target_msg.distance = 0.0;
    target_pub_->publish(target_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<front_aimer_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::pair<float, float>> points;
  sensor_msgs::msg::LaserScan laser_msg_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontAimer>());
  rclcpp::shutdown();
  return 0;
}