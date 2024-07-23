#include <chrono>
#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <thread>

using namespace std::chrono_literals;

constexpr double LIMIT_LEFT = 1.0;
constexpr double LIMIT_RIGHT = 10.0;
constexpr double LIMIT_TOP = 10.0;
constexpr double LIMIT_BOTTOM = 1.0;

class MotionController : public rclcpp::Node
{
public:
    MotionController() : Node("topics_practice_b"), distance(0.05), in_spiral(true), current_pose(nullptr)
    {
        start_countdown();
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MotionController::pose_callback, this, std::placeholders::_1));
        
        teleport_turtle(5.0, 5.0, 0.0);

        // Clear the screen
        clear_screen();

        timer_ = this->create_wall_timer(100ms, std::bind(&MotionController::move_turtle, this));
    }

private:
    void start_countdown()
    {
        for (int i = 5; i > 0; --i)
        {
            std::cout << "Application starts in: " << i << std::endl;
            std::this_thread::sleep_for(1s);
        }
        std::cout << "Initiating MotionController" << std::endl;
    }

    void teleport_turtle(double x, double y, double theta)
    {
        auto client = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        while (!client->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Teleport service not available, waiting again...");
        }
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto result = client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
        if (result.get() == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call teleport service");
        }
    }

    void clear_screen()
    {
        auto client = this->create_client<std_srvs::srv::Empty>("clear");
        while (!client->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Clear service not available, waiting again...");
        }
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose = msg;
    }

    void move_turtle()
    {
        if (!current_pose)
        {
            return;
        }

        auto vel_msg = geometry_msgs::msg::Twist();
        double x = current_pose->x;
        double y = current_pose->y;

        if (is_near_wall(x, y))
        {
            vel_msg.angular.z = 5.0;
            vel_msg.linear.x = 3.8;
            RCLCPP_INFO(this->get_logger(), "Avoiding walls");
            in_spiral = false;
        }
        else if (in_spiral)
        {
            vel_msg.linear.x = distance;
            vel_msg.angular.z = 2.0;
            distance += 0.05;
            RCLCPP_INFO(this->get_logger(), "Drawing spiral");
            if (x < LIMIT_LEFT || x > LIMIT_RIGHT || y > LIMIT_TOP || y < LIMIT_BOTTOM)
            {
                RCLCPP_INFO(this->get_logger(), "Going straight");
                in_spiral = false;
            }
        }
        else
        {
            vel_msg.linear.x = 4.0;
            RCLCPP_INFO(this->get_logger(), "Going straight");
        }

        publisher_->publish(vel_msg);
    }

    bool is_near_wall(double x, double y)
    {
        return (x - 1 < LIMIT_LEFT || x + 1 > LIMIT_RIGHT || y - 1 < LIMIT_BOTTOM || y + 1 > LIMIT_TOP);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double distance;
    bool in_spiral;
    turtlesim::msg::Pose::SharedPtr current_pose;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto motion_controller = std::make_shared<MotionController>();
    rclcpp::spin(motion_controller);
    rclcpp::shutdown();
    return 0;
}
