#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace std::placeholders;

class TurtleServicePractice : public rclcpp::Node
{
public:
  TurtleServicePractice()
    : Node("service_practice")
  {
    client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&TurtleServicePractice::pose_callback, this, _1));
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    if (msg->x > 5.5)
    {
      if (current_color_ != "red") {
        change_color(255, 0, 0); // Red
        RCLCPP_INFO(this->get_logger(), "Color: Red");
        current_color_ = "red";
      }
    }
    else
    {
      if (current_color_ != "green") {
        change_color(0, 255, 0); // Green
        RCLCPP_INFO(this->get_logger(), "Color: Green");
        current_color_ = "green";
      }
    }
  }

  void change_color(uint8_t r, uint8_t g, uint8_t b)
  {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = 3;
    request->off = 0;

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto future_result = client_->async_send_request(request);
    future_result.wait();  // Wait for the result
    if (future_result.valid()) {
      try
      {
        auto response = future_result.get();
        RCLCPP_INFO(this->get_logger(), "Service call succeeded.");
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
  }

  std::string current_color_ = "none"; // Track the current color
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleServicePractice>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
