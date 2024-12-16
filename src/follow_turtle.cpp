#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <vector>
#include <cmath>

class FollowerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
    std::string leader_frame_, follower_frame_;
    bool goal_reached_;

    struct PIDParams {
        double kp, ki, kd;
    };

    PIDParams linear_pid_{4.0, 0.0, 0.5};
    PIDParams angular_pid_{4.0, 0.0, 1.0};
    double linear_error_previous_, angular_error_previous_;
    double linear_error_integral_, angular_error_integral_;

    struct RelativePose {
        double x, y, theta;
    };

    RelativePose desired_pose_{0.0, 0.0, 0.0};

    double normalize_angle(double angle) const
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    double compute_pid(double error, double& error_integral, double& error_previous, const PIDParams& params)
    {
        error_integral += error;
        double derivative = error - error_previous;
        error_previous = error;
        return params.kp * error + params.ki * error_integral + params.kd * derivative;
    }

    void control_loop()
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = transform_buffer_->lookupTransform(follower_frame_, leader_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
            return;
        }

        double error_x = transform.transform.translation.x - desired_pose_.x;
        double error_y = transform.transform.translation.y - desired_pose_.y;
        double linear_error = hypot(error_x, error_y);
        double target_theta = atan2(error_y, error_x);
        double angular_error = normalize_angle(target_theta - desired_pose_.theta);

        if (linear_error < 0.1)
        {
            goal_reached_ = true;
            publish_velocity(0.0, 0.0);
            return;
        }

        goal_reached_ = false;

        double linear_speed = compute_pid(linear_error, linear_error_integral_, linear_error_previous_, linear_pid_);
        double angular_speed = compute_pid(angular_error, angular_error_integral_, angular_error_previous_, angular_pid_);

        double max_linear_speed = 2.0;
        double max_angular_speed = 2.0;

        linear_speed = std::clamp(linear_speed, -max_linear_speed, max_linear_speed);
        angular_speed = std::clamp(angular_speed, -max_angular_speed, max_angular_speed);

        publish_velocity(linear_speed, angular_speed);
    }

    void publish_velocity(double linear, double angular)
    {
        geometry_msgs::msg::Twist velocity_message;
        velocity_message.linear.x = linear;
        velocity_message.angular.z = angular;
        velocity_publisher_->publish(velocity_message);
        RCLCPP_INFO(this->get_logger(), "Velocity published: linear=%f, angular=%f", linear, angular);
    }

public:
    FollowerNode(const std::string& leader_frame, const std::string& follower_frame, double relative_x, double relative_y, double relative_theta)
        : Node("follower_node"),
          leader_frame_(leader_frame), follower_frame_(follower_frame),
          desired_pose_{relative_x, relative_y, relative_theta},
          goal_reached_(false),
          linear_error_previous_(0.0), angular_error_previous_(0.0),
          linear_error_integral_(0.0), angular_error_integral_(0.0)
    {
        transform_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*transform_buffer_);

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(follower_frame + "/cmd_vel", 10);
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FollowerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "FollowerNode for %s -> %s initialized", leader_frame.c_str(), follower_frame.c_str());
    }

    ~FollowerNode()
    {
        RCLCPP_INFO(this->get_logger(), "FollowerNode destroyed");
    }
};

class EventLoop
{
public:
    EventLoop(int argc, char* argv[], const std::string& leader_frame, const std::vector<std::string>& followers)
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        std::vector<FollowerNode::RelativePose> relative_poses = {
            {1.0, 0.0, 0.0}, {0.5, 0.5, M_PI / 4}, {0.5, -0.5, -M_PI / 4}
        };

        for (size_t i = 0; i < followers.size(); ++i)
        {
            auto node = std::make_shared<FollowerNode>(
                leader_frame, followers[i],
                relative_poses[i].x, relative_poses[i].y, relative_poses[i].theta);
            executor.add_node(node);
        }

        executor.spin();
    }

    ~EventLoop()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: follow leader follower1 [follower2 ...]");
        return 1;
    }

    std::string leader_frame = argv[1];
    std::vector<std::string> followers(argv + 2, argv + argc);

    EventLoop(argc, argv, leader_frame, followers);
    return 0;
}
