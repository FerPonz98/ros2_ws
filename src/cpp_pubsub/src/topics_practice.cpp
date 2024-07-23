#include <chrono>
#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

using namespace std::chrono_literals;

constexpr double LIMITE_IZQUIERDO = 0.5;
constexpr double LIMITE_DERECHO = 10.5;
constexpr double LIMITE_SUPERIOR = 10.5;
constexpr double LIMITE_INFERIOR = 0.5;

class MotionController : public rclcpp::Node
{
public:
    MotionController() : Node("motion_controller"), distancia(0.05), es_perform_spiral(true), current_pose(nullptr)
    {
        for (int i = 5; i > 0; --i)
        {
            std::cout << "Application starts in: " << i << std::endl;
            std::this_thread::sleep_for(1s);
        }
        std::cout << "Initiating MotionController" << std::endl;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MotionController::pose_callback, this, std::placeholders::_1));
        
        teleport_turtle(5.0, 5.0, 0.0);

        // Clear the screen
        clear_screen();

        timer_ = this->create_wall_timer(100ms, std::bind(&MotionController::mover_tortuga, this));
    }

private:
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

    void mover_tortuga()
    {
        if (!current_pose)
        {
            return;
        }

        auto vel_msg = geometry_msgs::msg::Twist();

        double x = current_pose->x;
        double y = current_pose->y;

        if (es_perform_spiral)
        {
            vel_msg.linear.x = distancia;
            vel_msg.angular.z = 1.0;
            distancia += 0.01;
            RCLCPP_INFO(this->get_logger(), "Drawing spiral");
        }
        else
        {
            vel_msg.linear.x = 2.0;
            vel_msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Going straight");
        }

        publisher_->publish(vel_msg);

        if (es_perform_spiral)
        {
            if (x < LIMITE_IZQUIERDO || x > LIMITE_DERECHO || y > LIMITE_SUPERIOR || y < LIMITE_INFERIOR)
            {
                RCLCPP_INFO(this->get_logger(), "Going straight");
                es_perform_spiral = false;
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double distancia;
    bool es_perform_spiral;
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
